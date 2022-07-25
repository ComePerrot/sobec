///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_ANTICIPATED_STATE_HPP_
#define SOBEC_ANTICIPATED_STATE_HPP_

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/core/state-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "sobec/fwd.hpp"

namespace sobec {

using namespace crocoddyl;

/**
 * @brief State residual
 *
 * This residual function defines the state tracking as
 * \f$\mathbf{r}=\mathbf{x}\ominus\mathbf{x}^*\f$, where
 * \f$\mathbf{x},\mathbf{x}^*\in~\mathcal{X}\f$ are the current and reference
 * states, respectively, which belong to the state manifold \f$\mathcal{X}\f$.
 * Note that the dimension of the residual vector is obtained from
 * `StateAbstract::get_ndx()`. Furthermore, the Jacobians of the residual
 * function are computed analytically.
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its
 * derivatives are calculated by `calc` and `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelAnticipatedStateTpl
    : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataAnticipatedStateTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef ActivationModelAbstractTpl<Scalar> ActivationModelAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the state residual model
   *
   * @param[in] state       State of the multibody system
   * @param[in] xref        Reference state
   * @param[in] nu          Dimension of the control vector
   */
  ResidualModelAnticipatedStateTpl(
      boost::shared_ptr<typename Base::StateAbstract> state,
      const std::size_t nu, const double& anticipated_time);

  /**
   * @brief Initialize the state residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state       State of the multibody system
   * @param[in] xref        Reference state
   */
  ResidualModelAnticipatedStateTpl(
      boost::shared_ptr<typename Base::StateAbstract> state,
      const double& anticipated_time);

  virtual ~ResidualModelAnticipatedStateTpl();

  /**
   * @brief Compute the state residual
   *
   * @param[in] data  State residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the Jacobians of the state residual
   *
   * @param[in] data  State residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Print relevant information of the state residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;

 private:
  MatrixXs time_matrix_;  //!< Reference state
};

template <typename _Scalar>
struct ResidualDataAnticipatedStateTpl
    : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::MatrixXs MatrixXs;

  template <template <typename Scalar> class Model>
  ResidualDataAnticipatedStateTpl(Model<Scalar>* const model,
                                  DataCollectorAbstract* const data)
      : Base(model, data),
        Jx(model->get_state()->get_ndx(), model->get_state()->get_ndx()) {
    Jx.setZero();
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d =
        dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  MatrixXs Jx;                            //!< Local Jacobian

  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/anticipated-state.hxx"

#endif  // CROCODDYL_MULTIBODY_RESIDUALS_ANTICIPATED_STATE_HPP_
