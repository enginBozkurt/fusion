#include "opt_pitch.h"

namespace g2o
{
    void EdgeLane::linearizeOplus()
    {
        const VertexSO3Mod *vso3 = static_cast<const VertexSO3Mod *>(_vertices[0]);
        const VertexVal *vx = static_cast<const VertexVal *>(_vertices[1]);
        const VertexVal2 *vab = static_cast<const VertexVal2 *>(_vertices[2]);
        Eigen::Vector2d ab = vab->estimate();
        double w = vx->estimate();
        Sophus::SO3d Rwc = vso3->estimate();

        Eigen::Vector3d vw, uv3;
        uv3 << uv[0], uv[1], 1.0;
        uv3 = Rwc * uv3;
        vw = uv3 / uv3[2];

        double a = vw[1];
        double z = h / a;
        double x = vw[0] * z;

        // dw
        _jacobianOplus[1](0, 0) = -1.0;

        // dab
        _jacobianOplus[2](0, 0) = -z * z;
        _jacobianOplus[2](0, 1) = -z;

        //dRSO3mod
        double dz = vw[0] - 2 * ab[0] * z - ab[1];

        Eigen::Matrix<double, 1, 2> dw;
        dw(0, 0) = z;
        dw(0, 1) = dz * (-h / (vw[1] * vw[1]));

        double invw2 = 1.0 / uv3[2];
        double invw22 = invw2 * invw2;
        Eigen::MatrixXd dvw = Eigen::MatrixXd::Zero(2, 3);
        dvw << invw2, 0.0, -uv3[0] * invw22,
            0.0, invw2, -uv3[1] * invw22;

        Eigen::Matrix3d dR = -Rwc.matrix() * Sophus::SO3d::hat(uv3);
        _jacobianOplus[0] = (dw * dvw * dR).block<1, 2>(0, 0);
    }
}; // namespace g2o