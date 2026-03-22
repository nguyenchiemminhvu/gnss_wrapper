// MIT License
//
// Copyright (c) 2026 nguyenchiemminhvu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <cstdint>
#include <memory>
#include <utility>

namespace gnss
{

/// Command: "set_datum <id>"
///
/// Programs the chip's NAVSPG-USRDAT configuration keys with the ellipsoid
/// parameters for the well-known datum identified by <id>.
/// <id> is the numeric value of ubx::parser::datum_id:
///   1 = WGS-84   2 = GRS-80   3 = PZ-90.11   4 = Krassovsky-1940
///   5 = CGCS2000  6 = Bessel-1841   7 = Clarke-1866
///
/// Returns false if the id has no known parameters or the transport fails.

class set_datum_command final : public gnss::i_gnss_command
{
public:
    set_datum_command(std::shared_ptr<gnss::i_gnss_controller> ctrl, uint8_t datum_id)
        : ctrl_(std::move(ctrl))
        , datum_id_(datum_id)
    {}

    bool execute() override { return ctrl_->set_datum(datum_id_); }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
    uint8_t                                  datum_id_;
};

} // namespace gnss
