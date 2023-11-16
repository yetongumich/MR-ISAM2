#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {
  class Pose2Point2Factor: public NoiseModelFactor2<Pose2, Point2> {
 private:
  typedef Pose2Point2Factor This;
  typedef NoiseModelFactor2<Pose2, Point2> Base;
  Point2 measured_;

 public:
  /** default constructor - only use for serialization */
  Pose2Point2Factor() {}

  /** Constructor */
  Pose2Point2Factor(Key key1, Key key2, const Point2& measured,
      const SharedNoiseModel& model = nullptr) :
    Base(model, key1, key2), measured_(measured) {
  }

  ~Pose2Point2Factor() {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "Pose2Point2Factor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ")\n";
    traits<Point2>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }

  /// assert equality up to a tolerance
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  /// @}
  /// @name NoiseModelFactor2 methods 
  /// @{

  /// evaluate error, returns vector of errors size of tangent space
  Vector evaluateError(const Pose2& pose, const Point2& point, boost::optional<Matrix&> H1 =
    boost::none, boost::optional<Matrix&> H2 = boost::none) const override {
    Point2 predicted_point = pose.transformTo(point, H1, H2);
    return predicted_point - measured_;
  }

  /// @}
  /// @name Standard interface 
  /// @{

  /// return the measurement
  const Point2& measured() const {
    return measured_;
  }
  /// @}

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }

	  // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
	  enum { NeedsToAlign = (sizeof(Point2) % 16) == 0 };
    public:
      GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  };
}