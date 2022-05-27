//
// Copyright (c) 2004-2022 Paul Ranson. paul@epicyclism.com
//
// Refer to licence in repository.
//
// useful gpiod constants and functions
//

#pragma once

#include <string_view>
#include <gpiod.hpp>

constexpr std::string_view app_name { "ed_reader"};
constexpr std::string_view chip_name { "gpiochip0"};
constexpr unsigned line_i  { 17 }; // pin 11 on connector
constexpr unsigned line_q  { 27 }; // pin 13
constexpr unsigned line_id { 22 }; // pin 15
constexpr size_t id_i   { 0 }; // offsets into the bulk line vector
constexpr size_t id_q   { 1 }; 
constexpr size_t id_idx { 2 }; 
constexpr int    pts_per_rev {500};

template<int LR, typename... Args> auto initialise(Args&&... args)
{
    try
    {
		gpiod::chip chip(chip_name.data());
		std::vector<unsigned> offsets;
        (offsets.emplace_back(std::forward<Args>(args)), ...);
		auto lines = chip.get_lines(offsets);
		lines.request({app_name.data(), LR, gpiod::line_request::FLAG_BIAS_PULL_UP});
		return lines;
    }
	catch(const std::exception& e)
	{
		std::cerr << "Error initialising gpio : " << e.what() << '\n';
	}
	return gpiod::line_bulk{};
}
