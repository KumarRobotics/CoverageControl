/**
 * A struct for 2D vector
 **/

#ifndef COVERAGECONTROL_VEC2D_H_
#define COVERAGECONTROL_VEC2D_H_

#include <cmath>
#include <ostream>

#include "constants.h"

namespace coveragecontrol {

	class Vec2d {

		private:
			double x_;
			double y_;

		public:

			Vec2d(): x_{0}, y_{0}{}
			Vec2d(double const x_i, double const y_i): x_{x_i}, y_{y_i}{}

			double x() const { return x_; }
			double y() const { return y_; }

			void SetX(double x) { x_ = x; }
			void SetY(double y) { y_ = y; }

			/*! Computes perpendicular Vector */
			Vec2d Perpendicular() const{
				Vec2d v_perpendicular;
				v_perpendicular = Vec2d(-y_, x_);
				return v_perpendicular;
			}

			/*! Adds two vectors */
			void Add(Vec2d const &v) {
				x_ += v.x(); y_ += v.y();
			}

			/*! Divide vector by a scalar */
			int Divide(const double scalar) {
				if(std::abs(scalar) < kEps) {
					return 1;
				}
				x_ = x_/scalar; y_ = y_/scalar;
				return 0;
			}

			/*! Computes dot product of two Vectors */
			double Dot(Vec2d const &v) const{
				return v.x() * x_ + v.y() * y_;
			}

			/*! Returns square of Euclidean distance from origin */
			double NormSqr() const{
				return x_ * x_ + y_ *y_;
			}

			/*! Returns Euclidean distance from origin */
			double Norm() const{
				return std::sqrt(NormSqr());
			}

			/*! Gives cosine of the angle between this and Vector v */
			int CosAngle(Vec2d const &v, double &ang) const{
				if (std::abs(Norm()) < 1e-10 || std::abs(v.Norm()) < 1e-10)
					return 1;
				ang = Dot(v)/(Norm() * v.Norm());
				return 0;
			}

			/*! Gives the distance between the Vector and another Vector v */
			double DistSqr(Vec2d const &v) const{
				double del_x = x_ - v.x();
				double del_y = y_ - v.y();
				double dSqr = (del_x * del_x + del_y * del_y);
				return dSqr;
			}

			double Dist(Vec2d const &v) const {
				return std::sqrt(DistSqr(v));
			}

			/*! Computes distance and angle with another Vector (v-this)*/
			void DistTht(Vec2d const &v, double &d, double &tht) const{
				d = std::sqrt(DistSqr(v));
				double del_x = -x_ + v.x();
				double del_y = -y_ + v.y();
				tht = std::atan2(del_y, del_x);
				if(tht < 0)
					tht += M_PI;
			}

			Vec2d operator+ (Vec2d const &vec) const {
				return Vec2d(x_ + vec.x(), y_ + vec.y());
			}

			Vec2d operator- (Vec2d const &vec) const { 
				return Vec2d(x_ - vec.x(), y_ - vec.y());
			}

			Vec2d operator- () const {
				return Vec2d(-x_, -y_);
			}

			Vec2d operator/ (double const &scalar) const {
				return Vec2d(x_ / scalar, y_ / scalar);
			}

			Vec2d operator* (double const &scalar) const {
				return Vec2d(x_ * scalar, y_ * scalar);
			}

			int Normalize () {
				double norm = Norm();
				if(norm < kEps) {
					return 1;
				}
				x_ = x_/norm;
				y_ = y_/norm;
				return 0;
			}
	};

} /* namespace coveragecontrol */
#endif /* COVERAGECONTROL_VEC2D_H_ */
