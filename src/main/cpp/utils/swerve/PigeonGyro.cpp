#include "utils/swerve/PigeonGyro.h"

#include <frc/Timer.h>

#include <cmath>
#include <utility>
#include <numbers>

#include "utils/Util.h"

using namespace ctre::phoenix::sensors;
using namespace hb;


PigeonGyro::PigeonGyro(int ID) {
  pigeon = new ctre::phoenix::sensors::PigeonIMU(ID);
  pigeon->ConfigFactoryDefault();
  m_offset = 0;
}

double PigeonGyro::GetAngle() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    PigeonIMU::FusionStatus stat;
    pigeon->GetFusedHeading(stat);
    m_angle = stat.heading;
  } 
  return m_angle + m_offset;
}

double PigeonGyro::GetRate() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    double rate[3];
    pigeon->GetRawGyro(rate);
    m_rate = rate[2];
  } 
  return m_rate;
}

void PigeonGyro::Reset() {
  pigeon->SetFusedHeading(0, 30);
  m_angle = m_rate = 0;
}

double PigeonGyro::GetPitch() {
  return pigeon->GetPitch();
}

double PigeonGyro::GetRoll() {
  return pigeon->GetRoll();
}

void PigeonGyro::Calibrate() {} // Gyro::Calibrate() is pure virtual

frc::Rotation2d PigeonGyro::GetRot2d() {
  return frc::Rotation2d(units::degree_t(GetAngle()));
}

units::radian_t PigeonGyro::GetRad() const {
  return units::radian_t((std::numbers::pi * GetAngle()) / 180);
}

void PigeonGyro::SetPosition(units::degree_t angle) {
  m_offset = angle.value();
}

double PigeonGyro::GetCompassHeading() const {

  double angle = GetAngle();
  int initSign = hb::sgn(angle);

  // Check if the angle is already in range
  if (fabs(angle) < 180) return angle;

  // add or subtract until the angle switches sign
  // for loop is used because while loop is not allowed
  // 30 is the recursion maximum which translates to roughly 30 turn in one direction before breaking
  for (int i = 0; i < 30; i++) {
    angle -= initSign * 360;
    if (fabs(angle) <= 180) {
      break;
    }
  }

  printf("Angle: %5.2f\n", angle);

  return std::lround(angle) % 360;

  // return angle;

}

void PigeonGyro::Set(units::degree_t heading) {
  int err = pigeon->SetFusedHeading(heading.value(), 30);
  m_angle = heading.value();
  m_rate = 0;
  if (err != 0) printf("Set Position Error\n");
}