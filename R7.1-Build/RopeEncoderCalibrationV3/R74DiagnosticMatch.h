#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace r74
{
    static const double kPi = 3.1415926535897932384626433832795;
    static const double kMaxControllerDMatchMm = 0.02;
    static const double kMaxControllerAlphaMatchDeg = 0.02;

    struct DiagnosticMatch
    {
        bool valid;
        bool accepted;
        double maxDMm;
        double maxAlphaDeg;
        double normalizedScore;

        DiagnosticMatch()
            : valid(false), accepted(false), maxDMm(std::numeric_limits<double>::infinity()),
              maxAlphaDeg(std::numeric_limits<double>::infinity()),
              normalizedScore(std::numeric_limits<double>::infinity())
        {
        }
    };

    inline DiagnosticMatch ScoreDiagnostic(
        const std::vector<double>& controllerDmm,
        const std::vector<double>& controllerAlphaLegacyDeg,
        const std::vector<double>& solvedDM,
        const std::vector<double>& solvedAlphaCraigRad)
    {
        DiagnosticMatch result;
        if (controllerDmm.size() < 7 || controllerAlphaLegacyDeg.size() < 7 ||
            solvedDM.size() < 7 || solvedAlphaCraigRad.size() < 7)
            return result;

        result.maxDMm = 0.0;
        for (int joint = 1; joint <= 4; ++joint)
        {
            result.maxDMm = (std::max)(result.maxDMm,
                std::fabs(controllerDmm[static_cast<std::size_t>(joint)] -
                    solvedDM[static_cast<std::size_t>(joint)] * 1000.0));
        }

        result.maxAlphaDeg = 0.0;
        for (int physicalAlpha = 0; physicalAlpha < 6; ++physicalAlpha)
        {
            result.maxAlphaDeg = (std::max)(result.maxAlphaDeg,
                std::fabs(controllerAlphaLegacyDeg[static_cast<std::size_t>(physicalAlpha)] -
                    solvedAlphaCraigRad[static_cast<std::size_t>(physicalAlpha + 1)] * 180.0 / kPi));
        }

        result.valid = std::isfinite(result.maxDMm) && std::isfinite(result.maxAlphaDeg);
        if (!result.valid)
            return result;

        result.normalizedScore = (std::max)(
            result.maxDMm / kMaxControllerDMatchMm,
            result.maxAlphaDeg / kMaxControllerAlphaMatchDeg);
        result.accepted = result.maxDMm <= kMaxControllerDMatchMm &&
            result.maxAlphaDeg <= kMaxControllerAlphaMatchDeg;
        return result;
    }

    inline bool BetterMatch(const DiagnosticMatch& candidate, const DiagnosticMatch& current)
    {
        if (!candidate.valid) return false;
        if (!current.valid) return true;
        return candidate.normalizedScore < current.normalizedScore;
    }
}
