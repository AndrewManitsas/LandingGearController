#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "intelligence_domain/model_rules.hpp"

int main()
{
  constexpr size_t NUM_WARMUP = 10000;
  constexpr size_t NUM_ITERATIONS = 1000000;

  // Representative synthetic test vectors matching the 3 world regimes
  const std::vector<std::vector<float>> test_cases = {
    // 0: SAFE (Alpha - planar rigid ground)
    {0.80f, 0.0001f, 0.002f, 0.0001f, 0.02f, 0.01f, 0.01f},
    // 1: UNSAFE_SLOPE (Beta - 20 deg ramp)
    {0.75f, 0.0150f, 0.010f, 0.0020f, 0.35f, 0.05f, 0.34f},
    // 2: UNSAFE_VEGETATION (Gamma - canopy elevation dispersion)
    {0.70f, 0.0080f, -0.090f, 0.0060f, 0.05f, 0.03f, 0.02f}
  };

  // Warmup run to prime instruction and data caches
  volatile int32_t dummy = 0;
  for (size_t i = 0; i < NUM_WARMUP; ++i) {
    const auto & vec = test_cases[i % test_cases.size()];
    dummy += intelligence_domain::predict_terrain(vec.data());
  }

  std::vector<double> latencies_ns;
  latencies_ns.reserve(NUM_ITERATIONS);

  // Timed inference benchmark
  for (size_t i = 0; i < NUM_ITERATIONS; ++i) {
    const auto & vec = test_cases[i % test_cases.size()];
    auto start = std::chrono::high_resolution_clock::now();
    int32_t verdict = intelligence_domain::predict_terrain(vec.data());
    auto end = std::chrono::high_resolution_clock::now();
    dummy += verdict;

    double elapsed_ns = std::chrono::duration<double, std::nano>(end - start).count();
    latencies_ns.push_back(elapsed_ns);
  }

  std::sort(latencies_ns.begin(), latencies_ns.end());

  double sum = std::accumulate(latencies_ns.begin(), latencies_ns.end(), 0.0);
  double mean_ns = sum / static_cast<double>(NUM_ITERATIONS);
  double min_ns = latencies_ns.front();
  double max_ns = latencies_ns.back();
  double p50_ns = latencies_ns[static_cast<size_t>(NUM_ITERATIONS * 0.50)];
  double p95_ns = latencies_ns[static_cast<size_t>(NUM_ITERATIONS * 0.95)];
  double p99_ns = latencies_ns[static_cast<size_t>(NUM_ITERATIONS * 0.99)];

  std::cout << "========================================================\n";
  std::cout << "  EDGE AI INFERENCE LATENCY BENCHMARK (" << NUM_ITERATIONS << " runs)\n";
  std::cout << "========================================================\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  Min Latency:     " << std::setw(8) << min_ns << " ns\n";
  std::cout << "  Mean Latency:    " << std::setw(8) << mean_ns << " ns\n";
  std::cout << "  Median (P50):    " << std::setw(8) << p50_ns << " ns\n";
  std::cout << "  95th Percentile: " << std::setw(8) << p95_ns << " ns\n";
  std::cout << "  99th Percentile: " << std::setw(8) << p99_ns << " ns\n";
  std::cout << "  Max Latency:     " << std::setw(8) << max_ns << " ns\n";
  std::cout << "--------------------------------------------------------\n";
  std::cout << "  Estimated Execution Frequency: > "
            << static_cast<uint64_t>(1e9 / (mean_ns > 0.0 ? mean_ns : 1.0)) << " Hz\n";
  std::cout << "========================================================\n";

  return 0;
}
