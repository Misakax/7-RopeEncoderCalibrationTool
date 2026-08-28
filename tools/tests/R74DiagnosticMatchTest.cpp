#include <cmath>
#include <iostream>
#include <vector>
#include "R74DiagnosticMatch.h"

static double deg(double value) { return value * r74::kPi / 180.0; }

static int Check(bool condition, const char* message)
{
    if (condition) return 0;
    std::cerr << "FAIL: " << message << std::endl;
    return 1;
}

int main()
{
    int failures = 0;
    const std::vector<double> controllerD = {0.0, 48.528, 492.559, -120.102, 316.970, 0.0, 93.0};
    const std::vector<double> controllerAlpha = {90.145, -89.974, 89.948, -89.850, 89.720, -90.085, 0.0};

    // This represents the older first-run Diagnostic that actually produced the
    // parameters persisted in the controller. Controller values are quantized.
    const std::vector<double> matchingD = {0.0, 0.0485284, 0.4925587, -0.1201016, 0.3169702, 0.0, 0.142};
    const std::vector<double> matchingAlpha = {0.0, deg(90.1454), deg(-89.9737), deg(89.9482), deg(-89.8496), deg(89.7196), deg(-90.0852)};
    const r74::DiagnosticMatch matching = r74::ScoreDiagnostic(
        controllerD, controllerAlpha, matchingD, matchingAlpha);
    failures += Check(matching.valid, "matching Diagnostic must be valid");
    failures += Check(matching.accepted, "controller-quantized Diagnostic must be accepted");
    failures += Check(matching.maxDMm < 0.001, "matching D error must stay below 0.001 mm");
    failures += Check(matching.maxAlphaDeg < 0.001, "matching Alpha error must stay below 0.001 deg");

    // This represents a newer calculation that was never written to the robot.
    // R7.3 picked this file by mtime and then rejected it. R7.4 must prefer the
    // older controller-matching file above regardless of recency.
    const std::vector<double> newerWrongD = {0.0, 0.048800, 0.492900, -0.119700, 0.317200, 0.0, 0.142};
    const std::vector<double> newerWrongAlpha = {0.0, deg(90.300), deg(-89.900), deg(90.100), deg(-89.700), deg(89.900), deg(-89.900)};
    const r74::DiagnosticMatch newerWrong = r74::ScoreDiagnostic(
        controllerD, controllerAlpha, newerWrongD, newerWrongAlpha);
    failures += Check(newerWrong.valid, "newer wrong Diagnostic must still be parse-valid");
    failures += Check(!newerWrong.accepted, "newer wrong Diagnostic must fail controller matching");
    failures += Check(r74::BetterMatch(matching, newerWrong), "older controller-matching Diagnostic must outrank newer wrong Diagnostic");
    failures += Check(!r74::BetterMatch(newerWrong, matching), "recency must not outrank controller-model match");

    const std::vector<double> malformed(3, 0.0);
    const r74::DiagnosticMatch invalid = r74::ScoreDiagnostic(
        controllerD, controllerAlpha, malformed, matchingAlpha);
    failures += Check(!invalid.valid, "incomplete Diagnostic arrays must be invalid");
    failures += Check(!invalid.accepted, "incomplete Diagnostic arrays must never be accepted");

    if (failures == 0)
        std::cout << "R74DiagnosticMatchTest PASS" << std::endl;
    return failures == 0 ? 0 : 1;
}
