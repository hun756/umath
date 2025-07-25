#include <umath/umath.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>


using namespace umath;

class FinancialCalculator
{
private:
    using MoneyType = Math<double>;

public:
    static double calculateCompoundInterest(double principal, double rate, int periods)
    {
        try
        {
            double base = MoneyType::addExact(1.0, rate);
            return MoneyType::multiplyExact(principal,
                                            MoneyType::pow(base, static_cast<double>(periods)));
        }
        catch (const arithmetic_overflow& e)
        {
            throw std::runtime_error("Interest calculation overflow: " + std::string(e.what()));
        }
    }

    static double calculatePresentValue(double futureValue, double discountRate, int periods)
    {
        double denominator =
            MoneyType::pow(MoneyType::addExact(1.0, discountRate), static_cast<double>(periods));
        return futureValue / denominator;
    }

    static double calculateBlackScholesCallPrice(double S,
                                                 double K,
                                                 double r,
                                                 double T,
                                                 double sigma)
    {
        double d1 = (MoneyType::log10(S / K) / MoneyType::log10(MoneyType::exp(1.0))
                     + (r + 0.5 * sigma * sigma) * T)
                    / (sigma * MoneyType::sqrt(T));
        double d2 = d1 - sigma * MoneyType::sqrt(T);

        double N_d1 = 0.5 * (1.0 + MoneyType::tanh(d1 / MoneyType::sqrt(2.0)));
        double N_d2 = 0.5 * (1.0 + MoneyType::tanh(d2 / MoneyType::sqrt(2.0)));

        return S * N_d1 - K * MoneyType::exp(-r * T) * N_d2;
    }

    static std::vector<double> calculateAmortizationSchedule(double principal,
                                                             double rate,
                                                             int periods)
    {
        std::vector<double> payments;
        payments.reserve(periods);

        double monthlyRate = rate / 12.0;
        double denominator = 1.0 - MoneyType::pow(1.0 + monthlyRate, -static_cast<double>(periods));
        double monthlyPayment = principal * monthlyRate / denominator;

        double remainingBalance = principal;

        for (int i = 0; i < periods; ++i)
        {
            double interestPayment = remainingBalance * monthlyRate;
            double principalPayment = monthlyPayment - interestPayment;
            remainingBalance = MoneyType::subtractExact(remainingBalance, principalPayment);
            payments.push_back(MoneyType::abs(remainingBalance));
        }

        return payments;
    }

    static double calculateVolatility(const std::vector<double>& prices)
    {
        if (prices.size() < 2)
            return 0.0;

        std::vector<double> returns;
        returns.reserve(prices.size() - 1);

        for (size_t i = 1; i < prices.size(); ++i)
        {
            double logReturn =
                MoneyType::log10(prices[i] / prices[i - 1]) / MoneyType::log10(MoneyType::exp(1.0));
            returns.push_back(logReturn);
        }

        double mean = 0.0;
        for (double ret : returns)
        {
            mean += ret;
        }
        mean /= returns.size();

        double variance = 0.0;
        for (double ret : returns)
        {
            double diff = ret - mean;
            variance += diff * diff;
        }
        variance /= (returns.size() - 1);

        return MoneyType::sqrt(variance * 252.0);
    }
};

int main()
{
    try
    {
        double principal = 100000.0;
        double rate = 0.05;
        int years = 10;

        double futureValue = FinancialCalculator::calculateCompoundInterest(principal, rate, years);
        std::cout << "Future Value: $" << Math<double>::round(futureValue * 100.0) / 100.0
                  << std::endl;

        double presentValue = FinancialCalculator::calculatePresentValue(futureValue, rate, years);
        std::cout << "Present Value: $" << Math<double>::round(presentValue * 100.0) / 100.0
                  << std::endl;

        double optionPrice =
            FinancialCalculator::calculateBlackScholesCallPrice(100.0, 105.0, 0.03, 0.25, 0.2);
        std::cout << "Option Price: $" << Math<double>::round(optionPrice * 100.0) / 100.0
                  << std::endl;

        std::vector<double> prices = {100.0, 102.5, 98.7, 105.2, 103.8, 107.1, 104.9};
        double volatility = FinancialCalculator::calculateVolatility(prices);
        std::cout << "Annualized Volatility: " << Math<double>::round(volatility * 10000.0) / 100.0
                  << "%" << std::endl;

        auto schedule = FinancialCalculator::calculateAmortizationSchedule(200000.0, 0.04, 360);
        std::cout << "Final Balance: $" << Math<double>::round(schedule.back() * 100.0) / 100.0
                  << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
