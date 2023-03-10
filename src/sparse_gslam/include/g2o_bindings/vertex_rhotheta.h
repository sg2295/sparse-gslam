#pragma once
#include <Eigen/Core>
#include <vector>

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/stuff/misc.h"

namespace g2o {
class EdgeSE2RhoTheta;
class VertexRhoTheta : public BaseVertex<2, Eigen::Vector2d> {
   public:
    Eigen::Vector2f start, end;
    double dist;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexRhoTheta() = default;
    void updateEndpoints();
    virtual void setToOriginImpl() override;
    virtual void oplusImpl(const double* update) override;
    virtual bool read(std::istream& is) override;
    virtual bool write(std::ostream& os) const override;
    void save(std::ostream& os) const {
        Eigen::Vector2d v = estimate();
        os << start[0] << ' ' << start[1] << ' ' << end[0] << ' ' << end[1] << ' ' << dist << ' ' << _id << ' ' << v[0] << ' ' << v[1] << '\n';
    }
    void load(std::istream& is) {
        Eigen::Vector2d v;
        is >> start[0] >> start[1] >> end[0] >> end[1] >> dist >> _id >> v[0] >> v[1];
        setEstimate(v);
    }
};
}  // namespace g2o
