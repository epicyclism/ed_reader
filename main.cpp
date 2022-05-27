//
// Copyright (c) 2004-2022 Paul Ranson. paul@epicyclism.com
//
// Refer to licence in repository.
//

#include <iostream>
#include <string>
#include <string_view>
#include <array>
#include <vector>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <numbers>
#include <utility>

#include "gpio_help.h"
#include "fftlib.h"
#include "basic_file.h"

using namespace std::literals;
constexpr double gain { 10.0 };

// returns duration, converts vector into interval times
//
std::pair<int64_t, std::vector<int64_t>> process(std::vector<int64_t> const& data)
{
	std::vector<int64_t> rv;
	rv.resize(data.size());
	auto duration = data.back() - data.front();
	std::adjacent_difference(data.begin(), data.end(), rv.begin());
	rv.erase(rv.begin());
	return {duration, rv};
}

auto prepare_for_fft(std::vector<int64_t> const& v, double angle_inc, double rpm)
{
	std::vector<fp_t> rv(v.size());
	// convert to rpm
	const auto factor = (angle_inc / (2.0 * std::numbers::pi)) * 60000000000;
	std::transform(v.begin(), v.end(), rv.begin(), [=](auto i){ return factor / i;});
	// convert to radians/sec
	//std::transform(v.begin(), v.end(), rv.begin(),[angle_inc](auto i){ return angle_inc * 1000000000 / i;});
	// remove dc
	std::transform(rv.begin(), rv.end(), rv.begin(), [=](auto i){ return i - rpm;});
	// amplify
	std::transform(rv.begin(), rv.end(), rv.begin(), [](auto v){ return v * gain;});
	for(int n = 0; n < 10; ++n)
		std::cout << "# " << rv[n] << "\n";
	return rv;
}

auto do_the_fft(std::vector<fp_t> const& data, int fft_size) -> std::pair<size_t, std::vector<fp_t>>
{
	auto pfft = make_fft(fft_size, window_t::HAMMING);
	auto wd   = pfft->width();
	std::vector<fp_t> mean(wd);
	if( data.size() < wd)
		return {pfft->width(), mean}; // no future
	// 50% overlap
	auto nffts = data.size() / wd * 2 - 1;
	std::cout << "# nffts = " << nffts << "\n";
	for (auto n = 0; n < nffts; ++n)
	{
		auto b = data.data() + n * wd / 2;
		auto[ob, oe] = (*pfft) ( b, b + wd);
		// add to average totals
		std::transform(mean.begin(), mean.end(), ob, mean.begin(), std::plus<>());
	}
	using namespace std::placeholders;
	// divide through
	std::transform(mean.begin(), mean.end(), mean.begin(), std::bind(std::divides<fp_t>(), _1, fp_t(nffts)));

	return {pfft->width(), mean};
}

// vector of timer values, count of expected steps per rev
//
void process_and_report(std::vector<int64_t>& data, int pts)
{
	auto[duration, intervals] = process(data);
	std::cout << "# duration for " << intervals.size() << " samples = " << duration << "\n";
	auto rpm = double(60000000000 * intervals.size() / pts) / double(duration);
	std::cout << "# rpm = " << rpm << "\n";
	auto av = std::accumulate(intervals.begin(), intervals.end(), 0LL) / (intervals.size());
	std::cout << "# av  = " << av << "\n";
	std::cout << "# expected = " << 3 * 60000000000LL / 100 / pts << "\n";
	std::cout << "# rpm = " << double(60000000000LL) / (av * pts) << "\n";
	std::cout << "# gain = " << gain << "\n";
	double angle_inc {std::numbers::pi * 2.0 / pts};
	// write the polar data
	double angle { 0.0 };
	// transform from time to velocity (rpm)
	std::vector<fp_t> velocities (intervals.size());
    // av is nanoseconds for angle_inc, so angular velocity is angle_inc / (av / 1 000 000 000) radians/sec
//		auto factor = angle_inc * 1000000000 ;
	// and rpm is (angle_inc / 2pi ) * 60000000000 / av
	auto factor = (angle_inc / (2.0 * std::numbers::pi)) * 60000000000;
	std::transform(intervals.begin(), intervals.end(), velocities.begin(), [=](auto i){ return factor / i ; });
	// amplify the variation by subracting rpm, multiplying, and putting rpm back
	std::transform(velocities.begin(), velocities.end(), velocities.begin(), [rpm](auto v){ return (v - rpm) * gain + rpm;});
	std::for_each(velocities.begin(), velocities.end(), [&angle, angle_inc](auto v){ std::cout << angle << " " << v << "\n"; angle += angle_inc;});
	// separate
	std::cout << "\n\n";
	// fft
	auto normalised_rpm = prepare_for_fft(intervals, angle_inc, rpm);
	auto [fft_width, fft_result] = do_the_fft(normalised_rpm, 13);
	// the 'sample rate' is the number of points times the rpm, reduced to Hertz
	auto sr = double(rpm * pts) / 60;
	// so each bucket is centred on sample_rate/fft_size
	sr /= fft_width; // make this a parameter.
	double fb {0};
	std::for_each(fft_result.begin(), fft_result.begin() + fft_result.size() / 2, [&fb, sr](auto f){ std::cout << fb << " " << f << "\n"; fb += sr;});
}

void proc_single_ms(auto& lines, auto n, auto comment)
{
	std::vector<int64_t> data ( 10000 );
	int cnt { 0 };
	auto line = lines[n];
	auto dd = data.begin();
	while(cnt < data.size())
	{
		if(line.event_wait(1000ms))
		{
			auto e = line.event_read();
			*dd = e.timestamp.count();
			++dd;
			++cnt;
		}
		else
			break;
	}
	std::cout << "# for polar, set polar, set size square, plot '...dat', for FFt, unset polar, plot '....dat' index 1\n";
	std::cout << "# " << comment << "\n";
	process_and_report(data, pts_per_rev);
}

int main(int ac, char** av)
{
	auto lines = initialise<gpiod::line_request::EVENT_RISING_EDGE>(line_i, line_q);
	for(auto& l : lines)
	{
		std::cout << "# " << l.name() << " " << l.offset() << " " << l.direction() << " " << l.bias() << "\n";
	}
	if(ac > 1)
		proc_single_ms(lines, id_i, av[1]);
	else
		proc_single_ms(lines, id_i, "");
}
