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
std::vector<float> v_filtered_t;
std::vector<float> v_original_alt;
std::vector<float> v_filtered_alt;
std::vector<float> v_filtered_vel;
std::vector<float> v_earth_rel_accel;
float t_apogee;
float alt_apogee;

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
        v_original_alt.push_back(ft);
    }
}

void RunFilter()
{
    v_filtered_t.reserve(entries.size());
    v_filtered_alt.reserve(entries.size());
    v_filtered_vel.reserve(entries.size());
    v_earth_rel_accel.reserve(entries.size());
    v_filtered_t.clear();
    v_filtered_alt.clear();
    v_filtered_vel.clear();
    v_earth_rel_accel.clear();

    auto ft = pressure_mbar_to_ft(entries.at(0).ms5607_pressure_mbar);
    auto m = ft * 0.3048;
    AltimeterFilterInit(m, 0.);


    for (int i = 0; i < STATE_LEN; i++)
    {
        AltimeterFilterSetProcessVariancePreApogee(i, process_variance_pre_apogee[i]);
        AltimeterFilterSetProcessVariancePostApogee(i, process_variance_post_apogee[i]);
    }

    for (int i = 0; i < OBSERVATION_LEN; i++)
    {
        AltimeterFilterSetObservationVariance(i, observation_variance[i]);
    }

    float avg_raw_accel_z = 0.0;
    for (int i = 0; i < 100; i++)
    {
        auto raw_accel_z = entries.at(i).adxl375_accel_z_mps2;
        assert(std::abs(raw_accel_z) < 20);
        avg_raw_accel_z += raw_accel_z;
    }
    avg_raw_accel_z /= 100.0;
    float accel_z_bias = avg_raw_accel_z - 9.81;
    // printf("Accel z bias: %f\n", accel_z_bias);

    t_apogee = 0.0;

    auto t0_ms = entries.at(0).time_boot_ms;
    for (const auto &e : entries)
    {
        auto ft = pressure_mbar_to_ft(e.ms5607_pressure_mbar);
        auto m = ft * 0.3048;

        auto earth_rel_accel = (e.adxl375_accel_z_mps2 - accel_z_bias) - G;
        auto filter_out = AltimeterFilterProcess(m, earth_rel_accel);
        auto ft_filtered = filter_out.altitude_m / 0.3048;

        auto t = (e.time_boot_ms - t0_ms) / 1000.0;
        v_filtered_t.push_back(t);
        v_filtered_alt.push_back(ft_filtered);
        v_filtered_vel.push_back(filter_out.velocity_mps);
        v_earth_rel_accel.push_back(earth_rel_accel);

        if (t_apogee == 0.0 && AltimeterFilterGetFlightStage() == STAGE_APOGEE)
        {
            t_apogee = t;
            alt_apogee = ft_filtered;
        }
    }

    assert(v_filtered_t.size() == v_filtered_alt.size());
    assert(v_original_alt.size() == v_filtered_alt.size());
    assert(v_earth_rel_accel.size() == v_filtered_alt.size());
    assert(v_filtered_vel.size() == v_filtered_alt.size());
}

void ShowVisualizer()
{
    RunFilter();

    ImVec2 size = ImGui::GetContentRegionAvail();
    size.y /= 3;
    size.y -= ImGuiStyleVar_ItemSpacing;

    if (ImGui::Begin("Plots"))
    {
        static ImPlotRect lims(0, 1, 0, 1);

        if (ImPlot::BeginAlignedPlots("AlignedGroup"))
        {
            if (ImPlot::BeginPlot("Altitude", size))
            {
                ImPlot::SetupAxes("Time (s)", "Altitude (ft)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Altitude, original", v_filtered_t.data(), v_original_alt.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Altitude, filtered", v_filtered_t.data(), v_filtered_alt.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::Annotation(t_apogee, alt_apogee, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Apogee");
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Velocity", size))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity (m/s)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Velocity (m/s)", v_filtered_t.data(), v_filtered_vel.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::Annotation(t_apogee, alt_apogee, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Apogee");
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Acceleration", size))
            {
                ImPlot::SetupAxes("Time (s)", "Acceleration (m/s^2)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Earth-Relative Acceleration (m/s^2)", v_filtered_t.data(), v_earth_rel_accel.data(), v_filtered_t.size(), ImPlotLineFlags_None);
                ImPlot::EndPlot();
            }
            ImPlot::EndAlignedPlots();
        }

        ImGui::End();
    }

    if (ImGui::Begin("Options")) {
        ImGui::Text("Process Variances");
        ImGui::SliderFloat("Altitude Pre-apogee", &process_variance_pre_apogee[0], 0.0f, 1.0f);
        ImGui::SliderFloat("Velocity Pre-apogee", &process_variance_pre_apogee[1], 0.0f, 1.0f);
        ImGui::SliderFloat("Acceleration Pre-apogee", &process_variance_pre_apogee[2], 0.0f, 1.0f);
        ImGui::SliderFloat("Jerk Pre-apogee", &process_variance_pre_apogee[3], 0.0f, 1.0f);

        ImGui::Separator();
        ImGui::Text("Observation Variances");
        ImGui::SliderFloat("Barometer", &observation_variance[0], 0.0f, 100);
        ImGui::SliderFloat("Accelerometer", &observation_variance[1], 0.0f, 100);

        ImGui::End();
    }
}