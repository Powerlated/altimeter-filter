// Main loop
#include "rapidcsv.h"
#include "airbrakes/AltimeterFilter.h"
#include "airbrakes/airbrakes.h"
#include <SDL3/SDL.h>
#include <implot.h>
#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <imgui_impl_sdl3.h>
#include <ranges>
#include <tuple>

struct DataEntry
{
    float time_boot_ms;
    float accel_z_mps2;
    float ms5607_pressure_mbar;
};

std::vector<DataEntry> entries;
std::vector<float> v_filtered_t;
std::vector<float> v_original_alt_ft;
std::vector<float> v_filtered_alt_ft;
std::vector<float> v_filtered_vel;
std::vector<float> v_predicted_apogee_ft;
std::vector<float> v_earth_rel_accel;
std::vector<float> v_filtered_accel;
std::vector<float> v_predicted_accel;
std::vector<float> v_apogee_ft_if_deployed_now;
float t_apogee;
float alt_apogee;
float t_burnout;
float alt_burnout;
float t_deploy;
float alt_deploy;
float t_retract;
float alt_retract;

extern float process_variance_pre_apogee[];
extern float process_variance_post_apogee[];
extern float observation_variance[];

extern float accel_z_bias;

extern float rocket_mass_kg;
extern float body_drag_model_scale;
extern float target_apogee_ft;

extern int airbrakes_stage;

bool graphOutOfDate = true;

void SetupVisualizer()
{
    ImPlot::CreateContext();

    rapidcsv::Document doc("test-flight-4-12-2025_flight-only.csv");

    for (const auto &[time_boot_ms,
                      accel_z_mps2,
                      ms5607_pressure_mbar] :
         std::views::zip(doc.GetColumn<float>("time_boot_ms"),
                         doc.GetColumn<float>("adxl375_accel_z_mps2_fc_frame"),
                         doc.GetColumn<float>("ms5607_pressure_mbar")))
    {
        entries.push_back({
            .time_boot_ms = time_boot_ms,
            .accel_z_mps2 = accel_z_mps2,
            .ms5607_pressure_mbar = ms5607_pressure_mbar,
        });

        auto ft = pressure_mbar_to_ft(ms5607_pressure_mbar);
        v_original_alt_ft.push_back(ft);
    }
}

void RunFilter()
{
    v_filtered_t.reserve(entries.size());
    v_filtered_accel.reserve(entries.size());
    v_filtered_alt_ft.reserve(entries.size());
    v_filtered_vel.reserve(entries.size());
    v_earth_rel_accel.reserve(entries.size());
    v_predicted_apogee_ft.reserve(entries.size());
    v_predicted_accel.reserve(entries.size());
    v_apogee_ft_if_deployed_now.reserve(entries.size());
    v_filtered_t.clear();
    v_filtered_alt_ft.clear();
    v_filtered_vel.clear();
    v_earth_rel_accel.clear();
    v_predicted_apogee_ft.clear();
    v_filtered_accel.clear();
    v_predicted_accel.clear();
    v_apogee_ft_if_deployed_now.clear();

    t_apogee = 0.0;
    alt_apogee = 0.0;
    t_burnout = 0.0;
    alt_burnout = 0.0;
    t_deploy = 0.0;
    alt_deploy = 0.0;
    t_retract = 0.0;
    alt_retract = 0.0;

    airbrakes_init();

    srand(1);

    auto t0_ms = entries.at(0).time_boot_ms;
    for (const auto &e : entries)
    {
        auto noised_pressure_mbar = e.ms5607_pressure_mbar;
        auto filter_out = airbrakes_process(noised_pressure_mbar, e.accel_z_mps2);
        airbrakes_check_for_retraction(filter_out);
        auto ft_filtered = filter_out.altitude_m / 0.3048;

        auto t = (e.time_boot_ms - t0_ms) / 1000.0;
        v_filtered_t.push_back(t);
        v_filtered_alt_ft.push_back(ft_filtered);
        v_filtered_vel.push_back(filter_out.velocity_mps);
        v_earth_rel_accel.push_back(0);
        v_filtered_accel.push_back(filter_out.acceleration_mps2);
        v_predicted_accel.push_back(0);

        if (t_apogee == 0.0 && AltimeterFilterGetFlightStage() == STAGE_APOGEE)
        {
            t_apogee = t;
            alt_apogee = ft_filtered;
        }

        if (t_burnout == 0.0 && AltimeterFilterGetFlightStage() == STAGE_BURNOUT)
        {
            t_burnout = t;
            alt_burnout = ft_filtered;
        }

        if (t_deploy == 0.0 && airbrakes_stage == AIRBRAKES_STAGE_DEPLOYED)
        {
            t_deploy = t;
            alt_deploy = ft_filtered;
        }

        if (t_retract == 0.0 && airbrakes_stage == AIRBRAKES_STAGE_RETRACTED)
        {
            t_retract = t;
            alt_retract = ft_filtered;
        }

        //
        // Apogee prediction
        if (AltimeterFilterGetFlightStage() == STAGE_BURNOUT)
        {
            v_predicted_apogee_ft.push_back(apogee_ft_if_stowed(filter_out.altitude_m, filter_out.velocity_mps));
            v_apogee_ft_if_deployed_now.push_back(apogee_ft_if_deployed_now(filter_out.altitude_m, filter_out.velocity_mps));
        }
        else
        {
            v_predicted_apogee_ft.push_back(0);
            v_apogee_ft_if_deployed_now.push_back(0);
        }
    }
}

void ShowVisualizer()
{
    if (graphOutOfDate)
    {
        graphOutOfDate = false;
        RunFilter();
    }

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
                ImPlot::PlotLine("Altitude, original", v_filtered_t.data(), v_original_alt_ft.data(), v_original_alt_ft.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Altitude, filtered", v_filtered_t.data(), v_filtered_alt_ft.data(), v_filtered_alt_ft.size(), ImPlotLineFlags_None);
                ImPlot::SetNextLineStyle(ImVec4(.99, .99, .2, 1));
                ImPlot::PlotLine("Predicted apogee", v_filtered_t.data(), v_predicted_apogee_ft.data(), v_predicted_apogee_ft.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Apogee if airbrake deployed now", v_filtered_t.data(), v_apogee_ft_if_deployed_now.data(), v_apogee_ft_if_deployed_now.size(), ImPlotLineFlags_None);
                float apogeeLineX[2] = {lims.X.Min, lims.X.Max};
                float apogeeLineY[2] = {alt_apogee, alt_apogee};
                ImPlot::SetNextLineStyle(ImVec4(.2, .8, .2, 1));
                ImPlot::PlotLine("Final apogee", apogeeLineX, apogeeLineY, 2, ImPlotLineFlags_None);
                ImPlot::Annotation(t_apogee, alt_apogee, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Apogee");
                ImPlot::Annotation(t_burnout, alt_burnout, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Burnout");
                ImPlot::Annotation(t_deploy, alt_deploy, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Airbrake Deploy");
                ImPlot::Annotation(t_retract, alt_retract, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Airbrake Retract");
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Velocity", size))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity (m/s)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Velocity (m/s)", v_filtered_t.data(), v_filtered_vel.data(), v_filtered_vel.size(), ImPlotLineFlags_None);
                ImPlot::Annotation(t_apogee, alt_apogee, ImPlot::GetLastItemColor(), ImVec2(10, 10), false, "Apogee");
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Acceleration", size))
            {
                ImPlot::SetupAxes("Time (s)", "Acceleration (m/s^2)");
                ImPlot::SetupAxisLinks(ImAxis_X1, &lims.X.Min, &lims.X.Max);
                ImPlot::PlotLine("Sensed Acceleration (m/s^2)", v_filtered_t.data(), v_earth_rel_accel.data(), v_earth_rel_accel.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Filtered Acceleration (m/s^2)", v_filtered_t.data(), v_filtered_accel.data(), v_filtered_accel.size(), ImPlotLineFlags_None);
                ImPlot::PlotLine("Predicted Acceleration (m/s^2)", v_filtered_t.data(), v_predicted_accel.data(), v_predicted_accel.size(), ImPlotLineFlags_None);
                ImPlot::EndPlot();
            }
            ImPlot::EndAlignedPlots();
        }

        ImGui::End();
    }

    if (ImGui::Begin("Options"))
    {
        ImGui::Text("Process Variances");
        graphOutOfDate |= ImGui::SliderFloat("Altitude Pre-apogee", &process_variance_pre_apogee[0], 0.0f, 1.0f);
        graphOutOfDate |= ImGui::SliderFloat("Velocity Pre-apogee", &process_variance_pre_apogee[1], 0.0f, 10.0f);
        graphOutOfDate |= ImGui::SliderFloat("Acceleration Pre-apogee", &process_variance_pre_apogee[2], 0.0f, 10.0f);
        graphOutOfDate |= ImGui::SliderFloat("Jerk Pre-apogee", &process_variance_pre_apogee[3], 0.0f, 10.0f);

        ImGui::Separator();
        ImGui::Text("Observation Variances");
        graphOutOfDate |= ImGui::SliderFloat("Barometer", &observation_variance[0], 0.0f, 100);
        graphOutOfDate |= ImGui::SliderFloat("Accelerometer", &observation_variance[1], 0.0f, 100);

        ImGui::Separator();
        ImGui::Text("Other");
        graphOutOfDate |= ImGui::SliderFloat("Rocket Mass (kg)", &rocket_mass_kg, 0.0f, 25);
        graphOutOfDate |= ImGui::SliderFloat("Body Drag Model Scale", &body_drag_model_scale, 0.5f, 20.0f);
        graphOutOfDate |= ImGui::SliderFloat("Target Apogee (ft)", &target_apogee_ft, 6000, 8000);
        graphOutOfDate |= ImGui::SliderFloat("Accelerometer Z Bias (m/s^2)", &accel_z_bias, 0, 12);

        ImGui::End();
    }
}