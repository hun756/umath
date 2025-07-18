#include <benchmark/benchmark.h>
#include <umath/umath.hpp>
static void BM_Dummy(benchmark::State& state) {
    for (auto _ : state) {
        // Simulate some work
        int a = 5;
    }
}

BENCHMARK(BM_Dummy);
