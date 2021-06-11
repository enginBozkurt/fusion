#ifndef OPTSCALE_H
#define OPTSCALE_H

#include "../core/base_vertex.h"
#include "../core/base_unary_edge.h"
#include "../core/base_binary_edge.h"
#include "../core/base_multi_edge.h"
#include <se3.hpp>
#include <so2.hpp>

namespace g2o
{
    class VertexVal : public BaseVertex<1, double>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexVal(){};
        virtual bool read(std::istream &is){};
        virtual bool write(std::ostream &os) const {};

        virtual void setToOriginImpl()
        {
            _estimate = 0.0;
        }

        virtual void oplusImpl(const double *update_)
        {
            setEstimate(update_[0] + estimate());

            return;
        }
    };

    class VertexVal2 : public BaseVertex<2, Eigen::Vector2d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexVal2(){};
        virtual bool read(std::istream &is){};
        virtual bool write(std::ostream &os) const {};

        virtual void setToOriginImpl()
        {
            _estimate << 0.0, 0.0;
        }

        virtual void oplusImpl(const double *update_)
        {
            Eigen::Vector2d update;
            update << update_[0], update_[1];
            setEstimate(update + estimate());

            return;
        }
    };

    class VertexSO3Mod : public BaseVertex<2, Sophus::SO3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSO3Mod(){};
        virtual bool read(std::istream &is){};
        virtual bool write(std::ostream &os) const {};

        virtual void setToOriginImpl()
        {
            _estimate = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, 0));
        }

        virtual void oplusImpl(const double *update)
        {
            Eigen::Vector3d omega;
            // omega << update[0], update[1], update[2];
            omega << update[0], update[1], 0.0;
            // omega << update[0], 0.0, 0.0;
            setEstimate(estimate() * Sophus::SO3d::exp(omega));

            return;
        }
    };

    class EdgeLane : public BaseMultiEdge<1, double>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeLane()
        {
            h = 1.2;
            resize(3);
        }

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError()
        {
            const VertexSO3Mod *vso3 = static_cast<const VertexSO3Mod *>(_vertices[0]);
            const VertexVal *vx = static_cast<const VertexVal *>(_vertices[1]);
            const VertexVal2 *vab = static_cast<const VertexVal2 *>(_vertices[2]);
            Eigen::Vector2d ab = vab->estimate();
            // std::cout << ab.transpose() << std::endl;

            Sophus::SO3d Rwc = vso3->estimate();
            Eigen::Vector3d vw;
            vw << uv[0], uv[1], 1.0;
            vw = Rwc * vw;
            vw /= vw[2];

            double a = vw[1];
            double z = h / a;
            double x = vw[0] * z;

            double obs = ab[0] * z * z + ab[1] * z + vx->estimate();
            _error[0] = x - obs;
        }

        virtual void linearizeOplus();

        Eigen::Vector2d uv;
        double h;
    };

} // namespace g2o

#endif