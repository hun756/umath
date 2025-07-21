#include "umath/vec2.hpp"

#include <benchmark/benchmark.h>

#include <cmath>
#include <random>
#include <vector>


using namespace umath;

class BenchmarkData
{
public:
    static BenchmarkData& GetInstance()
    {
        static BenchmarkData instance;
        return instance;
    }

    const std::vector<Vector2f>& GetVectors() const
    {
        return vectors_;
    }
    const std::vector<float>& GetScalars() const
    {
        return scalars_;
    }
    const std::vector<float>& GetAngles() const
    {
        return angles_;
    }

private:
    BenchmarkData()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> vec_dist(-100.0f, 100.0f);
        std::uniform_real_distribution<float> scalar_dist(0.1f, 10.0f);
        std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * PI);

        vectors_.reserve(VECTOR_COUNT);
        scalars_.reserve(VECTOR_COUNT);
        angles_.reserve(VECTOR_COUNT);

        for (size_t i = 0; i < VECTOR_COUNT; ++i)
        {
            Vector2f vec;
            do
            {
                vec = Vector2f(vec_dist(gen), vec_dist(gen));
            }
            while (vec.length_squared() < 0.001f);

            vectors_.push_back(vec);
            scalars_.push_back(scalar_dist(gen));
            angles_.push_back(angle_dist(gen));
        }
    }

    static constexpr size_t VECTOR_COUNT = 10'000;
    std::vector<Vector2f> vectors_;
    std::vector<float> scalars_;
    std::vector<float> angles_;
};

class Vector2Fixture : public benchmark::Fixture
{
public:
    void SetUp(const benchmark::State& state) override
    {
        data_ = &BenchmarkData::GetInstance();
    }

    void TearDown(const benchmark::State& state) override
    {
        // Nothing to clean up
    }

protected:
    const BenchmarkData* data_;
};

BENCHMARK_F(Vector2Fixture, VectorAddition)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = vectors[index] + vectors[(index + 1) % vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, VectorSubtraction)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = vectors[index] - vectors[(index + 1) % vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, ScalarMultiplication)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    const auto& scalars = data_->GetScalars();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = vectors[index] * scalars[index];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, VectorMultiplication)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = vectors[index] * vectors[(index + 1) % vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, ScalarDivision)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    const auto& scalars = data_->GetScalars();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = vectors[index] / scalars[index];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, LengthCalculation)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result = vectors[index].length();
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, LengthSquared)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result = vectors[index].length_squared();
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, FastLength)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result = vectors[index].fast_length();
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, Normalization)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = Vector2f::normalize(vectors[index]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, FastNormalization)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = Vector2f::normalize_fast(vectors[index]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, InPlaceNormalization)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        state.PauseTiming();
        Vector2f vec = vectors[index];  // Copy to avoid modifying original
        state.ResumeTiming();

        vec.normalize();
        benchmark::DoNotOptimize(vec);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, DotProduct)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result = Vector2f::dot(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, CrossProduct)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result = Vector2f::cross(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, EuclideanDistance)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result = Vector2f::distance(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, DistanceSquared)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result =
            Vector2f::distance_squared(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, FastDistance)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result =
            Vector2f::distance_fast(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, ManhattanDistance)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result =
            Vector2f::manhattan_distance(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, ChebyshevDistance)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float result =
            Vector2f::chebyshev_distance(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, StandardRotation)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    const auto& angles = data_->GetAngles();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = Vector2f::rotate(vectors[index], angles[index]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, FastRotation)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    const auto& angles = data_->GetAngles();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = Vector2f::rotate_fast(vectors[index], angles[index]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, RotationAroundPivot)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    const auto& angles = data_->GetAngles();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = Vector2f::rotate_around(vectors[index],
                                                  angles[index],
                                                  vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, LinearInterpolation)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float t = static_cast<float>(index % 100) / 100.0f;
        Vector2f result = Vector2f::lerp(vectors[index], vectors[(index + 1) % vectors.size()], t);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, SphericalLinearInterpolation)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float t = static_cast<float>(index % 100) / 100.0f;
        Vector2f result = Vector2f::slerp(vectors[index], vectors[(index + 1) % vectors.size()], t);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, SmoothStep)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        float t = static_cast<float>(index % 100) / 100.0f;
        Vector2f result =
            Vector2f::smooth_step(vectors[index], vectors[(index + 1) % vectors.size()], t);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, VectorConstruction)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f vec(vectors[index].x, vectors[index].y);
        benchmark::DoNotOptimize(vec);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, VectorCopy)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f copy(vectors[index]);
        benchmark::DoNotOptimize(copy);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, VectorAssignment)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    Vector2f target;
    size_t index = 0;

    for (auto _ : state)
    {
        target = vectors[index];
        benchmark::DoNotOptimize(target);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, HashCalculation)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        std::size_t hash_val = vectors[index].hash();
        benchmark::DoNotOptimize(hash_val);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, EqualityComparison)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        bool result = vectors[index] == vectors[(index + 1) % vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, ApproximateComparison)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        bool result = vectors[index].approximately_equals(vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

BENCHMARK_F(Vector2Fixture, LexicographicComparison)(benchmark::State& state)
{
    const auto& vectors = data_->GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        int result = Vector2f::compare(vectors[index], vectors[(index + 1) % vectors.size()]);
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}

static void BM_FloatVectorAddition(benchmark::State& state)
{
    const auto& data = BenchmarkData::GetInstance();
    const auto& vectors = data.GetVectors();
    size_t index = 0;

    for (auto _ : state)
    {
        Vector2f result = vectors[index] + vectors[(index + 1) % vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_FloatVectorAddition);

static void BM_DoubleVectorAddition(benchmark::State& state)
{
    const auto& data = BenchmarkData::GetInstance();
    const auto& float_vectors = data.GetVectors();

    static std::vector<Vector2d> double_vectors;
    if (double_vectors.empty())
    {
        double_vectors.reserve(float_vectors.size());
        for (const auto& vec : float_vectors)
        {
            double_vectors.emplace_back(static_cast<double>(vec.x), static_cast<double>(vec.y));
        }
    }

    size_t index = 0;

    for (auto _ : state)
    {
        Vector2d result =
            double_vectors[index] + double_vectors[(index + 1) % double_vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % double_vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_DoubleVectorAddition);

static void BM_IntVectorAddition(benchmark::State& state)
{
    const auto& data = BenchmarkData::GetInstance();
    const auto& float_vectors = data.GetVectors();

    static std::vector<Vector2i> int_vectors;
    if (int_vectors.empty())
    {
        int_vectors.reserve(float_vectors.size());
        for (const auto& vec : float_vectors)
        {
            int_vectors.emplace_back(static_cast<int>(vec.x), static_cast<int>(vec.y));
        }
    }

    size_t index = 0;

    for (auto _ : state)
    {
        Vector2i result = int_vectors[index] + int_vectors[(index + 1) % int_vectors.size()];
        benchmark::DoNotOptimize(result);
        index = (index + 1) % int_vectors.size();
    }

    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_IntVectorAddition);

static void RegisterComparativeBenchmarks()
{
    benchmark::RegisterBenchmark("Length_Standard",
                                 [](benchmark::State& state)
                                 {
                                     const auto& data = BenchmarkData::GetInstance();
                                     const auto& vectors = data.GetVectors();
                                     size_t index = 0;

                                     for (auto _ : state)
                                     {
                                         float result = vectors[index].length();
                                         benchmark::DoNotOptimize(result);
                                         index = (index + 1) % vectors.size();
                                     }
                                     state.SetItemsProcessed(state.iterations());
                                 });

    benchmark::RegisterBenchmark("Length_Fast",
                                 [](benchmark::State& state)
                                 {
                                     const auto& data = BenchmarkData::GetInstance();
                                     const auto& vectors = data.GetVectors();
                                     size_t index = 0;

                                     for (auto _ : state)
                                     {
                                         float result = vectors[index].fast_length();
                                         benchmark::DoNotOptimize(result);
                                         index = (index + 1) % vectors.size();
                                     }
                                     state.SetItemsProcessed(state.iterations());
                                 });

    benchmark::RegisterBenchmark("Normalize_Standard",
                                 [](benchmark::State& state)
                                 {
                                     const auto& data = BenchmarkData::GetInstance();
                                     const auto& vectors = data.GetVectors();
                                     size_t index = 0;

                                     for (auto _ : state)
                                     {
                                         Vector2f result = Vector2f::normalize(vectors[index]);
                                         benchmark::DoNotOptimize(result);
                                         index = (index + 1) % vectors.size();
                                     }
                                     state.SetItemsProcessed(state.iterations());
                                 });

    benchmark::RegisterBenchmark("Normalize_Fast",
                                 [](benchmark::State& state)
                                 {
                                     const auto& data = BenchmarkData::GetInstance();
                                     const auto& vectors = data.GetVectors();
                                     size_t index = 0;

                                     for (auto _ : state)
                                     {
                                         Vector2f result = Vector2f::normalize_fast(vectors[index]);
                                         benchmark::DoNotOptimize(result);
                                         index = (index + 1) % vectors.size();
                                     }
                                     state.SetItemsProcessed(state.iterations());
                                 });

    benchmark::RegisterBenchmark("Rotation_Standard",
                                 [](benchmark::State& state)
                                 {
                                     const auto& data = BenchmarkData::GetInstance();
                                     const auto& vectors = data.GetVectors();
                                     const auto& angles = data.GetAngles();
                                     size_t index = 0;

                                     for (auto _ : state)
                                     {
                                         Vector2f result =
                                             Vector2f::rotate(vectors[index], angles[index]);
                                         benchmark::DoNotOptimize(result);
                                         index = (index + 1) % vectors.size();
                                     }
                                     state.SetItemsProcessed(state.iterations());
                                 });

    benchmark::RegisterBenchmark("Rotation_Fast",
                                 [](benchmark::State& state)
                                 {
                                     const auto& data = BenchmarkData::GetInstance();
                                     const auto& vectors = data.GetVectors();
                                     const auto& angles = data.GetAngles();
                                     size_t index = 0;

                                     for (auto _ : state)
                                     {
                                         Vector2f result =
                                             Vector2f::rotate_fast(vectors[index], angles[index]);
                                         benchmark::DoNotOptimize(result);
                                         index = (index + 1) % vectors.size();
                                     }
                                     state.SetItemsProcessed(state.iterations());
                                 });

    benchmark::RegisterBenchmark(
        "Distance_Standard",
        [](benchmark::State& state)
        {
            const auto& data = BenchmarkData::GetInstance();
            const auto& vectors = data.GetVectors();
            size_t index = 0;

            for (auto _ : state)
            {
                float result =
                    Vector2f::distance(vectors[index], vectors[(index + 1) % vectors.size()]);
                benchmark::DoNotOptimize(result);
                index = (index + 1) % vectors.size();
            }
            state.SetItemsProcessed(state.iterations());
        });

    benchmark::RegisterBenchmark(
        "Distance_Fast",
        [](benchmark::State& state)
        {
            const auto& data = BenchmarkData::GetInstance();
            const auto& vectors = data.GetVectors();
            size_t index = 0;

            for (auto _ : state)
            {
                float result =
                    Vector2f::distance_fast(vectors[index], vectors[(index + 1) % vectors.size()]);
                benchmark::DoNotOptimize(result);
                index = (index + 1) % vectors.size();
            }
            state.SetItemsProcessed(state.iterations());
        });
}

static void BM_ComponentAccess(benchmark::State& state)
{
    Vector2f vec(3.14f, 2.71f);

    for (auto _ : state)
    {
        float x = vec.x;
        float y = vec.y;
        benchmark::DoNotOptimize(x);
        benchmark::DoNotOptimize(y);
    }

    state.SetItemsProcessed(state.iterations() * 2);  // Two accesses per iteration
}
BENCHMARK(BM_ComponentAccess);

static void BM_ArrayAccess(benchmark::State& state)
{
    Vector2f vec(3.14f, 2.71f);

    for (auto _ : state)
    {
        float x = vec[0];
        float y = vec[1];
        benchmark::DoNotOptimize(x);
        benchmark::DoNotOptimize(y);
    }

    state.SetItemsProcessed(state.iterations() * 2);
}
BENCHMARK(BM_ArrayAccess);

static void BM_StaticConstantAccess(benchmark::State& state)
{
    for (auto _ : state)
    {
        const Vector2f& zero = Vector2f::zero();
        const Vector2f& one = Vector2f::one();
        benchmark::DoNotOptimize(zero);
        benchmark::DoNotOptimize(one);
    }

    state.SetItemsProcessed(state.iterations() * 2);
}
BENCHMARK(BM_StaticConstantAccess);

int main(int argc, char** argv)
{
    benchmark::Initialize(&argc, argv);

    RegisterComparativeBenchmarks();

    benchmark::SetDefaultTimeUnit(benchmark::kNanosecond);

    benchmark::AddCustomContext("compiler",
#ifdef __clang__
                                "clang"
#elif defined(__GNUC__)
                                "gcc"
#elif defined(_MSC_VER)
                                "msvc"
#else
                                "unknown"
#endif
    );

    benchmark::AddCustomContext("build_type",
#ifdef DEBUG
                                "debug"
#else
                                "release"
#endif
    );

    benchmark::AddCustomContext("vector_count",
                                std::to_string(BenchmarkData::GetInstance().GetVectors().size()));

    if (benchmark::ReportUnrecognizedArguments(argc, argv))
    {
        return 1;
    }

    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();

    return 0;
}
