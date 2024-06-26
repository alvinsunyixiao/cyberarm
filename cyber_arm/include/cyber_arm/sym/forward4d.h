// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace symik {

/**
 * This function was autogenerated. Do not modify by hand.
 *
 * Args:
 *     q: Matrix41
 *     lamb: Scalar
 *     target4d: Matrix41
 *     l: Matrix41
 *
 * Outputs:
 *     f: Matrix41
 *     e: Scalar
 *     J: Matrix44
 *     A: Matrix44
 */
template <typename Scalar>
void Forward4D(const Eigen::Matrix<Scalar, 4, 1>& q, const Scalar lamb,
               const Eigen::Matrix<Scalar, 4, 1>& target4d, const Eigen::Matrix<Scalar, 4, 1>& l,
               Eigen::Matrix<Scalar, 4, 1>* const f = nullptr, Scalar* const e = nullptr,
               Eigen::Matrix<Scalar, 4, 4>* const J = nullptr,
               Eigen::Matrix<Scalar, 4, 4>* const A = nullptr) {
  // Total ops: 441

  // Input arrays

  // Intermediate terms (153)
  const Scalar _tmp0 = (Scalar(1) / Scalar(2)) * q(2, 0);
  const Scalar _tmp1 = std::sin(_tmp0);
  const Scalar _tmp2 = std::pow(_tmp1, Scalar(2));
  const Scalar _tmp3 = l(2, 0) * (1 - Scalar(2.0) * _tmp2);
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * q(1, 0);
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = std::sin(_tmp4);
  const Scalar _tmp7 = _tmp5 * _tmp6;
  const Scalar _tmp8 = (Scalar(1) / Scalar(2)) * q(0, 0);
  const Scalar _tmp9 = std::cos(_tmp8);
  const Scalar _tmp10 = std::sin(_tmp8);
  const Scalar _tmp11 = _tmp10 * _tmp9;
  const Scalar _tmp12 = Scalar(4.0) * _tmp11 * _tmp7;
  const Scalar _tmp13 = _tmp12 * _tmp3;
  const Scalar _tmp14 = _tmp12 * l(1, 0);
  const Scalar _tmp15 = std::cos(_tmp0);
  const Scalar _tmp16 = _tmp15 * _tmp5;
  const Scalar _tmp17 = _tmp16 * _tmp9;
  const Scalar _tmp18 = Scalar(1.0) * _tmp9;
  const Scalar _tmp19 = _tmp1 * _tmp6;
  const Scalar _tmp20 = _tmp17 + _tmp18 * _tmp19;
  const Scalar _tmp21 = Scalar(1.0) * _tmp10;
  const Scalar _tmp22 = _tmp21 * _tmp6;
  const Scalar _tmp23 = _tmp1 * _tmp5;
  const Scalar _tmp24 = _tmp15 * _tmp22 - _tmp21 * _tmp23;
  const Scalar _tmp25 = 2 * _tmp24;
  const Scalar _tmp26 = -_tmp1 * _tmp22 - _tmp16 * _tmp21;
  const Scalar _tmp27 = _tmp15 * _tmp6;
  const Scalar _tmp28 = _tmp18 * _tmp23 - _tmp18 * _tmp27;
  const Scalar _tmp29 = 2 * _tmp28;
  const Scalar _tmp30 = _tmp20 * _tmp25 + _tmp26 * _tmp29;
  const Scalar _tmp31 = (Scalar(1) / Scalar(2)) * q(3, 0);
  const Scalar _tmp32 = std::sin(_tmp31);
  const Scalar _tmp33 = std::pow(_tmp32, Scalar(2));
  const Scalar _tmp34 = l(3, 0) * (1 - Scalar(2.0) * _tmp33);
  const Scalar _tmp35 = 2 * _tmp26;
  const Scalar _tmp36 = -_tmp20 * _tmp35 + _tmp24 * _tmp29;
  const Scalar _tmp37 = std::cos(_tmp31);
  const Scalar _tmp38 = Scalar(2.0) * _tmp32 * _tmp37 * l(3, 0);
  const Scalar _tmp39 = std::pow(_tmp6, Scalar(2));
  const Scalar _tmp40 = Scalar(2.0) * _tmp39;
  const Scalar _tmp41 = _tmp11 * _tmp40;
  const Scalar _tmp42 = std::pow(_tmp5, Scalar(2));
  const Scalar _tmp43 = Scalar(2.0) * _tmp11 * _tmp42;
  const Scalar _tmp44 = -_tmp41 + _tmp43;
  const Scalar _tmp45 = Scalar(2.0) * _tmp1 * _tmp15 * l(2, 0);
  const Scalar _tmp46 = _tmp13 + _tmp14 + _tmp30 * _tmp34 + _tmp36 * _tmp38 - _tmp44 * _tmp45;
  const Scalar _tmp47 = std::pow(_tmp10, Scalar(2));
  const Scalar _tmp48 = Scalar(2.0) * _tmp47;
  const Scalar _tmp49 = Scalar(1.0) - _tmp48;
  const Scalar _tmp50 = Scalar(2.0) * _tmp7;
  const Scalar _tmp51 = _tmp50 * l(1, 0);
  const Scalar _tmp52 = _tmp48 * _tmp7;
  const Scalar _tmp53 = -_tmp52;
  const Scalar _tmp54 = std::pow(_tmp9, Scalar(2));
  const Scalar _tmp55 = _tmp50 * _tmp54;
  const Scalar _tmp56 = _tmp53 + _tmp55;
  const Scalar _tmp57 = _tmp20 * _tmp29;
  const Scalar _tmp58 = _tmp24 * _tmp35;
  const Scalar _tmp59 = -_tmp57 + _tmp58;
  const Scalar _tmp60 = 1 - 2 * std::pow(_tmp28, Scalar(2));
  const Scalar _tmp61 = -2 * std::pow(_tmp26, Scalar(2)) + _tmp60;
  const Scalar _tmp62 = -_tmp40 * _tmp54 + 1;
  const Scalar _tmp63 = -_tmp42 * _tmp48 + _tmp62;
  const Scalar _tmp64 =
      _tmp3 * _tmp56 + _tmp34 * _tmp59 + _tmp38 * _tmp61 - _tmp45 * _tmp63 + _tmp49 * _tmp51;
  const Scalar _tmp65 = -_tmp39 * _tmp48 + _tmp62;
  const Scalar _tmp66 = -_tmp55;
  const Scalar _tmp67 = _tmp53 + _tmp66;
  const Scalar _tmp68 = -2 * std::pow(_tmp24, Scalar(2)) + _tmp60;
  const Scalar _tmp69 = _tmp57 + _tmp58;
  const Scalar _tmp70 = Scalar(1.0) * l(1, 0);
  const Scalar _tmp71 = _tmp3 * _tmp65 + _tmp34 * _tmp68 + _tmp38 * _tmp69 - _tmp45 * _tmp67 +
                        _tmp70 * (1 - _tmp40) + l(0, 0);
  const Scalar _tmp72 = q(1, 0) - q(2, 0) + q(3, 0);
  const Scalar _tmp73 = Scalar(0.5) * std::pow(Scalar(_tmp46 - target4d(0, 0)), Scalar(2)) +
                        Scalar(0.5) * std::pow(Scalar(_tmp64 - target4d(1, 0)), Scalar(2)) +
                        Scalar(0.5) * std::pow(Scalar(_tmp71 - target4d(2, 0)), Scalar(2)) +
                        Scalar(0.5) * std::pow(Scalar(_tmp72 - target4d(3, 0)), Scalar(2));
  const Scalar _tmp74 = Scalar(1.0) * _tmp54;
  const Scalar _tmp75 = _tmp42 * _tmp74;
  const Scalar _tmp76 = _tmp39 * _tmp74;
  const Scalar _tmp77 = Scalar(1.0) * _tmp47;
  const Scalar _tmp78 = _tmp39 * _tmp77 - _tmp42 * _tmp77;
  const Scalar _tmp79 = _tmp75 - _tmp76 + _tmp78;
  const Scalar _tmp80 = Scalar(0.5) * _tmp9;
  const Scalar _tmp81 = _tmp27 * _tmp80;
  const Scalar _tmp82 = _tmp23 * _tmp80;
  const Scalar _tmp83 = _tmp81 - _tmp82;
  const Scalar _tmp84 = Scalar(0.5) * _tmp10;
  const Scalar _tmp85 = _tmp19 * _tmp84;
  const Scalar _tmp86 = -_tmp85;
  const Scalar _tmp87 = -Scalar(1) / Scalar(2) * _tmp10 * _tmp16 + _tmp86;
  const Scalar _tmp88 = _tmp27 * _tmp84;
  const Scalar _tmp89 = _tmp23 * _tmp84;
  const Scalar _tmp90 = _tmp88 - _tmp89;
  const Scalar _tmp91 = _tmp25 * _tmp90;
  const Scalar _tmp92 = Scalar(0.5) * _tmp17;
  const Scalar _tmp93 = _tmp19 * _tmp80;
  const Scalar _tmp94 = -_tmp92 - _tmp93;
  const Scalar _tmp95 = 2 * _tmp20;
  const Scalar _tmp96 = _tmp94 * _tmp95;
  const Scalar _tmp97 = _tmp91 - _tmp96;
  const Scalar _tmp98 =
      -_tmp3 * _tmp52 + _tmp3 * _tmp55 +
      _tmp34 * (_tmp25 * _tmp87 + _tmp29 * _tmp94 + _tmp35 * _tmp90 + _tmp83 * _tmp95) +
      _tmp38 * (_tmp29 * _tmp83 - _tmp35 * _tmp87 + _tmp97) - _tmp45 * _tmp79 - _tmp52 * l(1, 0) +
      _tmp55 * l(1, 0);
  const Scalar _tmp99 = _tmp90 * _tmp95;
  const Scalar _tmp100 = -_tmp99;
  const Scalar _tmp101 = _tmp29 * _tmp87;
  const Scalar _tmp102 = _tmp25 * _tmp94;
  const Scalar _tmp103 = _tmp102 + _tmp35 * _tmp83;
  const Scalar _tmp104 = 4 * _tmp26;
  const Scalar _tmp105 = 4 * _tmp28;
  const Scalar _tmp106 = -_tmp105 * _tmp90;
  const Scalar _tmp107 = -_tmp13 - _tmp14 + _tmp34 * (_tmp100 - _tmp101 + _tmp103) +
                         _tmp38 * (-_tmp104 * _tmp94 + _tmp106) - _tmp45 * (_tmp41 - _tmp43);
  const Scalar _tmp108 = 4 * _tmp24;
  const Scalar _tmp109 =
      _tmp34 * (_tmp106 - _tmp108 * _tmp83) + _tmp38 * (_tmp101 + _tmp103 + _tmp99);
  const Scalar _tmp110 = _tmp16 * _tmp84;
  const Scalar _tmp111 = _tmp110 + _tmp85;
  const Scalar _tmp112 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp113 = -_tmp112 * _tmp27 + _tmp82;
  const Scalar _tmp114 = Scalar(8.0) * _tmp10 * _tmp17 * _tmp19 * l(2, 0);
  const Scalar _tmp115 =
      _tmp114 - _tmp3 * _tmp41 + _tmp3 * _tmp43 +
      _tmp34 * (_tmp111 * _tmp95 + _tmp113 * _tmp25 + _tmp29 * _tmp90 + _tmp35 * _tmp94) +
      _tmp38 * (_tmp100 + _tmp102 + _tmp111 * _tmp29 - _tmp113 * _tmp35) - _tmp41 * l(1, 0) +
      _tmp43 * l(1, 0);
  const Scalar _tmp116 = -_tmp105 * _tmp94;
  const Scalar _tmp117 = _tmp111 * _tmp35;
  const Scalar _tmp118 = _tmp113 * _tmp29;
  const Scalar _tmp119 = _tmp49 * _tmp70;
  const Scalar _tmp120 = -_tmp119 * _tmp39 + _tmp119 * _tmp42 + _tmp3 * _tmp79 +
                         _tmp34 * (_tmp117 - _tmp118 + _tmp97) +
                         _tmp38 * (-_tmp104 * _tmp90 + _tmp116) - _tmp45 * (_tmp52 + _tmp66);
  const Scalar _tmp121 = _tmp3 * _tmp67 + _tmp34 * (-_tmp108 * _tmp111 + _tmp116) +
                         _tmp38 * (_tmp117 + _tmp118 + _tmp91 + _tmp96) -
                         _tmp45 * (-_tmp75 + _tmp76 + _tmp78) - _tmp51;
  const Scalar _tmp122 = -_tmp88 + _tmp89;
  const Scalar _tmp123 = -_tmp112 * _tmp23 + _tmp81;
  const Scalar _tmp124 = -_tmp110 + _tmp86;
  const Scalar _tmp125 = _tmp92 + _tmp93;
  const Scalar _tmp126 = 2 * _tmp125;
  const Scalar _tmp127 = Scalar(1.0) * l(2, 0);
  const Scalar _tmp128 = _tmp127 * _tmp2;
  const Scalar _tmp129 = _tmp127 * std::pow(_tmp15, Scalar(2));
  const Scalar _tmp130 =
      -_tmp114 + _tmp128 * _tmp44 - _tmp129 * _tmp44 +
      _tmp34 * (_tmp122 * _tmp29 + _tmp123 * _tmp25 + _tmp124 * _tmp95 + _tmp126 * _tmp26) +
      _tmp38 * (-_tmp122 * _tmp95 - _tmp123 * _tmp35 + _tmp124 * _tmp29 + _tmp126 * _tmp24);
  const Scalar _tmp131 = -_tmp105 * _tmp125;
  const Scalar _tmp132 = _tmp126 * _tmp20;
  const Scalar _tmp133 = _tmp123 * _tmp29;
  const Scalar _tmp134 = _tmp122 * _tmp25 + _tmp124 * _tmp35;
  const Scalar _tmp135 = _tmp128 * _tmp63 - _tmp129 * _tmp63 +
                         _tmp34 * (-_tmp132 - _tmp133 + _tmp134) +
                         _tmp38 * (-_tmp104 * _tmp122 + _tmp131) - _tmp45 * _tmp56;
  const Scalar _tmp136 = _tmp128 * _tmp67 - _tmp129 * _tmp67 +
                         _tmp34 * (-_tmp108 * _tmp124 + _tmp131) +
                         _tmp38 * (_tmp132 + _tmp133 + _tmp134) - _tmp45 * _tmp65;
  const Scalar _tmp137 = Scalar(1.0) * l(3, 0);
  const Scalar _tmp138 = _tmp137 * _tmp33;
  const Scalar _tmp139 = std::pow(_tmp37, Scalar(2));
  const Scalar _tmp140 = _tmp137 * _tmp139;
  const Scalar _tmp141 = -_tmp138 * _tmp36 + _tmp140 * _tmp36 - _tmp30 * _tmp38;
  const Scalar _tmp142 = -_tmp138 * _tmp61 + _tmp140 * _tmp61 - _tmp38 * _tmp59;
  const Scalar _tmp143 = _tmp137 * _tmp69;
  const Scalar _tmp144 = _tmp139 * _tmp143 - _tmp143 * _tmp33 - _tmp38 * _tmp68;
  const Scalar _tmp145 = _tmp73 + lamb;
  const Scalar _tmp146 = _tmp107 * _tmp120 + _tmp109 * _tmp121 + _tmp115 * _tmp98;
  const Scalar _tmp147 = _tmp107 * _tmp135 + _tmp109 * _tmp136 + _tmp130 * _tmp98;
  const Scalar _tmp148 = _tmp107 * _tmp142 + _tmp109 * _tmp144 + _tmp141 * _tmp98;
  const Scalar _tmp149 = _tmp145 + 1;
  const Scalar _tmp150 = _tmp115 * _tmp130 + _tmp120 * _tmp135 + _tmp121 * _tmp136 - 1;
  const Scalar _tmp151 = _tmp115 * _tmp141 + _tmp120 * _tmp142 + _tmp121 * _tmp144 + 1;
  const Scalar _tmp152 = _tmp130 * _tmp141 + _tmp135 * _tmp142 + _tmp136 * _tmp144 - 1;

  // Output terms (4)
  if (f != nullptr) {
    Eigen::Matrix<Scalar, 4, 1>& _f = (*f);

    _f(0, 0) = _tmp46;
    _f(1, 0) = _tmp64;
    _f(2, 0) = _tmp71;
    _f(3, 0) = _tmp72;
  }

  if (e != nullptr) {
    Scalar& _e = (*e);

    _e = _tmp73;
  }

  if (J != nullptr) {
    Eigen::Matrix<Scalar, 4, 4>& _J = (*J);

    _J(0, 0) = _tmp98;
    _J(1, 0) = _tmp107;
    _J(2, 0) = _tmp109;
    _J(3, 0) = 0;
    _J(0, 1) = _tmp115;
    _J(1, 1) = _tmp120;
    _J(2, 1) = _tmp121;
    _J(3, 1) = 1;
    _J(0, 2) = _tmp130;
    _J(1, 2) = _tmp135;
    _J(2, 2) = _tmp136;
    _J(3, 2) = -1;
    _J(0, 3) = _tmp141;
    _J(1, 3) = _tmp142;
    _J(2, 3) = _tmp144;
    _J(3, 3) = 1;
  }

  if (A != nullptr) {
    Eigen::Matrix<Scalar, 4, 4>& _A = (*A);

    _A(0, 0) = std::pow(_tmp107, Scalar(2)) + std::pow(_tmp109, Scalar(2)) + _tmp145 +
               std::pow(_tmp98, Scalar(2));
    _A(1, 0) = _tmp146;
    _A(2, 0) = _tmp147;
    _A(3, 0) = _tmp148;
    _A(0, 1) = _tmp146;
    _A(1, 1) = std::pow(_tmp115, Scalar(2)) + std::pow(_tmp120, Scalar(2)) +
               std::pow(_tmp121, Scalar(2)) + _tmp149;
    _A(2, 1) = _tmp150;
    _A(3, 1) = _tmp151;
    _A(0, 2) = _tmp147;
    _A(1, 2) = _tmp150;
    _A(2, 2) = std::pow(_tmp130, Scalar(2)) + std::pow(_tmp135, Scalar(2)) +
               std::pow(_tmp136, Scalar(2)) + _tmp149;
    _A(3, 2) = _tmp152;
    _A(0, 3) = _tmp148;
    _A(1, 3) = _tmp151;
    _A(2, 3) = _tmp152;
    _A(3, 3) = std::pow(_tmp141, Scalar(2)) + std::pow(_tmp142, Scalar(2)) +
               std::pow(_tmp144, Scalar(2)) + _tmp149;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace symik
