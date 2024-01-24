/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

/**
 * @file
 * @brief Exports the SIN_TABLE, used by the Angle class.
 */

#pragma once

/**
 * @namespace PolyFit::math
 * @brief PolyFit::math
 */
namespace PolyFit {
namespace math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

} // namespace math
} // namespace PolyFit
