// Main loop
#include "rapidcsv.h"
#include "AltimeterFilter.h"
#include <SDL3/SDL.h>
#include <implot.h>
#include <imgui_impl_sdl3.h>
#include <ranges>
#include <tuple>

struct DataEntry
{
    float time_boot_ms;
    float adxl375_accel_z_mps2;
    float ms5607_pressure_mbar;
};

std::vector<DataEntry> entries;
std::vector<float> filtered_t;
std::vector<float> original_alt;
std::vector<float> filtered_alt;

float process_variance_pre_apogee[] = {1., 1., 1., 1.};
float process_variance_post_apogee[] = {1., 1., 1., 1.};
float observation_variance[] = {1., 1.};

void SetupVisualizer()
{
    ImPlot::CreateContext();

    rapidcsv::Document doc("test-flight-4-12-2025_flight-only.csv");

    for (const auto &[time_boot_ms,
                      adxl375_accel_z_mps2_fc_frame,
                      ms5607_pressure_mbar] :
         std::views::zip(doc.GetColumn<float>("time_boot_ms"),
                         doc.GetColumn<float>("adxl375_accel_z_mps2_fc_frame"),
                         doc.GetColumn<float>("ms5607_pressure_mbar")))
    {
        entries.push_back({
            .time_boot_ms = time_boot_ms,
            .adxl375_accel_z_mps2 = adxl375_accel_z_mps2_fc_frame,
            .ms5607_pressure_mbar = ms5607_pressure_mbar,
        });

        auto ft = pressure_mbar_to_ft(ms5607_pressure_mbar);
        original_alt.push_back(ft);
    }
}

void RunFilter()
{
    filtered_t.reserve(entries.size());
    filtered_alt.reserve(entries.size());
    filtered_t.clear();
    filtered_alt.clear();

    auto ft = pressure_mbar_to_ft(entries.at(0).ms5607_pressure_mbar);
    auto m = ft * 0.3048;
    AltimeterFilterInit(m, 0.);

    for (int i = 0; i < STATE_LEN; i++) {
        AltimeterFilterSetProcessVariancePreApogee(i, process_variance_pre_apogee[i]);
        AltimeterFilterSetProcessVariancePostApogee(i, process_variance_post_apogee[i]);
    }

    for (int i = 0; i < OBSERVATION_LEN; i++) {
        AltimeterFilterSetObservationVariance(i, observation_variance[i]);
    }

    auto t0_ms = entries.at(0).time_boot_ms;
    for (const auto &e : entries)
    {
        auto ft = pressure_mbar_to_ft(e.ms5607_pressure_mbar);
        auto m = ft * 0.3048;

        auto filter_out = AltimeterFilterProcess(m);
        auto ft_filtered = filter_out.altitude_m / 0.3048;

        auto t = (e.time_boot_ms - t0_ms) / 1000.0;
        filtered_t.push_back(t);
        filtered_alt.push_back(ft_filtered);
    }

    assert(filtered_t.size() == filtered_alt.size());
    assert(original_alt.size() == filtered_alt.size());
}

void ShowVisualizer()
{
    RunFilter();

    ImVec2 size = ImGui::GetContentRegionAvail();

    if (ImPlot::BeginPlot("Line Plots", size))
    {
        ImPlot::SetupAxes("Time (s)", "Altitude (ft)");
        ImPlot::PlotLine("Altitude, original", filtered_t.data(), original_alt.data(), filtered_t.size(), ImPlotLineFlags_None);
        ImPlot::PlotLine("Altitude, filtered", filtered_t.data(), filtered_alt.data(), filtered_t.size(), ImPlotLineFlags_None);
        ImPlot::EndPlot();
    }
}