#include "pose_estimation.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
double bilinear_gray(const cv::Mat& img, double x, double y, bool& ok)
{
    if(x < 1.0 || y < 1.0 || x >= static_cast<double>(img.cols) - 2.0 || y >= static_cast<double>(img.rows) - 2.0) {
        ok = false;
        return 0.0;
    }
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const double fx = x - static_cast<double>(x0);
    const double fy = y - static_cast<double>(y0);
    const double v00 = img.at<uint8_t>(y0, x0);
    const double v10 = img.at<uint8_t>(y0, x0 + 1);
    const double v01 = img.at<uint8_t>(y0 + 1, x0);
    const double v11 = img.at<uint8_t>(y0 + 1, x0 + 1);
    ok = true;
    return (1.0 - fx) * (1.0 - fy) * v00 + fx * (1.0 - fy) * v10 + (1.0 - fx) * fy * v01 + fx * fy * v11;
}

// I(t) = a + b / (1 + exp(-k (t - t0)))
// 返回 t0；对比度不足或 t0 跑飞则失败。
bool fit_sigmoid_t0(const std::vector<double>& ts, const std::vector<double>& is, double& t0_out)
{
    if(ts.size() < 5) {
        return false;
    }

    double i_min = is[0];
    double i_max = is[0];
    for(double v : is) {
        i_min = std::min(i_min, v);
        i_max = std::max(i_max, v);
    }
    const double contrast = i_max - i_min;
    if(contrast < 8.0) {
        return false;
    }

    double a = i_min;
    double b = contrast;
    double k = 1.5;
    double t0 = 0.0;

    for(int iter = 0; iter < 8; ++iter) {
        Eigen::Matrix4d jtj = Eigen::Matrix4d::Zero();
        Eigen::Vector4d jtr = Eigen::Vector4d::Zero();

        for(size_t i = 0; i < ts.size(); ++i) {
            double u = k * (ts[i] - t0);
            u = std::max(-20.0, std::min(20.0, u));
            const double s = 1.0 / (1.0 + std::exp(-u));
            const double pred = a + b * s;
            const double r = pred - is[i];
            const double dsdu = s * (1.0 - s);
            Eigen::Vector4d jac;
            jac << 1.0, s, b * dsdu * (ts[i] - t0), b * dsdu * (-k);
            jtj += jac * jac.transpose();
            jtr += jac * r;
        }

        Eigen::Vector4d delta = jtj.ldlt().solve(jtr);
        if(!delta.allFinite()) {
            return false;
        }
        a -= delta(0);
        b -= delta(1);
        k -= delta(2);
        t0 -= delta(3);
        if(delta.norm() < 1e-4) {
            break;
        }
    }

    if(!std::isfinite(t0) || !std::isfinite(k)) {
        return false;
    }
    if(std::abs(k) < 0.3) {
        return false;
    }
    if(std::abs(t0) > 2.5) {
        return false;
    }
    t0_out = t0;
    return true;
}

bool fit_line_from_points(const std::vector<cv::Point2d>& pts, double& ex, double& ey, double& nx, double& ny)
{
    if(pts.size() < 3) {
        return false;
    }
    double mx = 0.0, my = 0.0;
    for(const auto& p : pts) {
        mx += p.x;
        my += p.y;
    }
    mx /= static_cast<double>(pts.size());
    my /= static_cast<double>(pts.size());

    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for(const auto& p : pts) {
        const double dx = p.x - mx;
        const double dy = p.y - my;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    const double n = static_cast<double>(pts.size());
    cxx /= n;
    cxy /= n;
    cyy /= n;

    // 最小特征值对应的方向为法向（与 AprilTag refine_edges 相同）
    const double theta = 0.5 * std::atan2(-2.0 * cxy, cyy - cxx);
    ex = mx;
    ey = my;
    nx = std::cos(theta);
    ny = std::sin(theta);
    return true;
}

bool intersect_lines(double ex0, double ey0, double nx0, double ny0,
                     double ex1, double ey1, double nx1, double ny1,
                     double& x, double& y)
{
    // n·(p - e) = 0
    Eigen::Matrix2d a;
    a << nx0, ny0, nx1, ny1;
    if(std::abs(a.determinant()) < 1e-6) {
        return false;
    }
    Eigen::Vector2d rhs;
    rhs << nx0 * ex0 + ny0 * ey0, nx1 * ex1 + ny1 * ey1;
    const Eigen::Vector2d p = a.inverse() * rhs;
    if(!p.allFinite()) {
        return false;
    }
    x = p(0);
    y = p(1);
    return true;
}
} // namespace

CornerRefineResult refine_corners_esf(const cv::Mat& gray, double p[4][2])
{
    CornerRefineResult result;
    if(gray.empty() || gray.type() != CV_8UC1) {
        return result;
    }

    double orig[4][2];
    for(int i = 0; i < 4; ++i) {
        orig[i][0] = p[i][0];
        orig[i][1] = p[i][1];
    }

    double lines[4][4]; // Ex, Ey, nx, ny
    bool line_ok[4] = {false, false, false, false};

    for(int edge = 0; edge < 4; ++edge) {
        const int a = edge;
        const int b = (edge + 1) & 3;
        const double dx = orig[b][0] - orig[a][0];
        const double dy = orig[b][1] - orig[a][1];
        const double mag = std::hypot(dx, dy);
        if(mag < 8.0) {
            continue;
        }

        // 与 AprilTag refine_edges 一致：法向 (dy, -dx)
        const double nx = dy / mag;
        const double ny = -dx / mag;
        const double range = (mag < 15.0) ? 4.0 : 3.0;
        const int nsamples = 12;

        std::vector<cv::Point2d> edge_pts;
        edge_pts.reserve(static_cast<size_t>(nsamples));

        for(int s = 0; s < nsamples; ++s) {
            result.n_sample_all++;
            // 避开角点，α ∈ (0.15, 0.85)
            const double alpha = 0.15 + 0.70 * (static_cast<double>(s) + 1.0) / (static_cast<double>(nsamples) + 1.0);
            const double x0 = (1.0 - alpha) * orig[a][0] + alpha * orig[b][0];
            const double y0 = (1.0 - alpha) * orig[a][1] + alpha * orig[b][1];

            std::vector<double> ts;
            std::vector<double> is;
            ts.reserve(17);
            is.reserve(17);
            for(double t = -range; t <= range + 1e-9; t += 0.5) {
                bool ok = false;
                const double v = bilinear_gray(gray, x0 + t * nx, y0 + t * ny, ok);
                if(!ok) {
                    continue;
                }
                ts.push_back(t);
                is.push_back(v);
            }

            double t0 = 0.0;
            if(!fit_sigmoid_t0(ts, is, t0)) {
                continue;
            }
            result.n_sample_ok++;
            edge_pts.emplace_back(x0 + t0 * nx, y0 + t0 * ny);
        }

        if(!fit_line_from_points(edge_pts, lines[edge][0], lines[edge][1], lines[edge][2], lines[edge][3])) {
            continue;
        }
        line_ok[edge] = true;
        result.n_edge_ok++;
    }

    if(result.n_edge_ok < 3) {
        return result;
    }

    double refined[4][2];
    int n_corner_ok = 0;
    for(int i = 0; i < 4; ++i) {
        refined[i][0] = orig[i][0];
        refined[i][1] = orig[i][1];
        const int e0 = (i + 3) & 3; // 进入该角点的边
        const int e1 = i;           // 离开该角点的边
        if(!line_ok[e0] || !line_ok[e1]) {
            continue;
        }
        double x = orig[i][0];
        double y = orig[i][1];
        if(!intersect_lines(lines[e0][0], lines[e0][1], lines[e0][2], lines[e0][3],
                            lines[e1][0], lines[e1][1], lines[e1][2], lines[e1][3],
                            x, y)) {
            continue;
        }
        // 亚像素修正不应远离 refine_edges 结果；超过 1.5px 视为失败，保留原角点
        if(std::hypot(x - orig[i][0], y - orig[i][1]) > 1.5) {
            continue;
        }
        refined[i][0] = x;
        refined[i][1] = y;
        n_corner_ok++;
    }

    if(n_corner_ok < 3) {
        return result;
    }

    for(int i = 0; i < 4; ++i) {
        p[i][0] = refined[i][0];
        p[i][1] = refined[i][1];
    }
    result.updated = true;
    return result;
}
