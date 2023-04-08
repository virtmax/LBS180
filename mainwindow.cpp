#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <stdio.h>

#define HOST "localhost"
#define PORT 4223
#define UID_HE "K7Z"    // the UID of your Hall Effect Bricklet 2.0
#define UID_SS "5WNYW6" // the UID of your Silent Stepper Brick

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // create a timer
    updateTimer = new QTimer(this);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(UpdateSlot()));
    updateTimer->start(100);

    connect(updateTimer, &QTimer::timeout, this, QOverload<>::of(&MainWindow::update));

    timeSeries.setTimeGranularity(1);

    fluxStepsTable.setNumOfColumns(2);

    QFont font;
    font.setFamily("Sans");
    font.setPixelSize(15);

    QCustomPlotZoom *customPlot = nullptr;

    customPlot = ui->plot_magnetic_flux;
    customPlot->legend->setVisible(false);
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::black));
    customPlot->yAxis->setLabel("magnetic flux (µT)");
    customPlot->xAxis->setLabel("time");
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTicks(false);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis->setTicks(true);
    customPlot->xAxis2->setTicks(false);
    customPlot->xAxis2->setVisible(true);
    customPlot->setZoomMode(true);


    customPlot = ui->plot_magnetic_flux_over_steps;
    customPlot->legend->setVisible(false);
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::black));
    customPlot->yAxis->setLabel("magnetic flux (µT)");
    customPlot->xAxis->setLabel("steps");
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTicks(false);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis->setTicks(true);
    customPlot->xAxis2->setTicks(false);
    customPlot->xAxis2->setVisible(true);
    customPlot->setZoomMode(true);
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::blue));
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::blue));
}

MainWindow::~MainWindow()
{
    hall_effect_v2_destroy(&he);
    silent_stepper_destroy(&ss);

    if(connected)
        ipcon_destroy(&ipcon);

    delete ui;
}


void MainWindow::UpdateSlot(void)
{
    QVector<double> x, y;
    int graphNr = 0;
    QCustomPlotZoom *customPlot = nullptr;

    customPlot = ui->plot_magnetic_flux;
    x.clear();y.clear();
    for(size_t i=0; i < timeSeries.nBins(); i++)
    {
        x.push_back(timeSeries.data(i).first);
        y.push_back(timeSeries.data(i).second);
    }
    customPlot->graph(graphNr)->setData(x, y);
    customPlot->rescaleAxes();
    customPlot->replot();

    customPlot = ui->plot_magnetic_flux_over_steps;
    x.clear();y.clear();
    for(size_t i=0; i < fluxStepsTable.getNumberOfRows(); i++)
    {
        const auto& row = fluxStepsTable.getData().at(i);
        x.push_back(row[0]);
        y.push_back(row[1]);
    }
    customPlot->graph(graphNr)->setData(x, y);
    customPlot->rescaleAxes();
    customPlot->replot();

    ui->l_degrees_per_step->setText(nucmath::to_string(degreesPerStep, 7).c_str());

    RotPosFrame *frame = ui->frame;
    frame->rotationAngle = rotationAngle;

    // Don't use device before ipcon is connected
    if(!connected)
        return;

    // Get current magnetic flux density
    int16_t magnetic_flux_density;
    if(hall_effect_v2_get_magnetic_flux_density(&he, &magnetic_flux_density) < 0)
    {
        std::cout << "Could not get magnetic flux density, probably timeout\n";
    }
    else
    {
        //std::cout << "Magnetic flux density: " << magnetic_flux_density << " µT" << std::endl;
        timeSeries.add(timeSeries.nBins(), magnetic_flux_density);
    }


    if(motorState == MotorState::calibRot)
    {
        int32_t remainingSteps = 0;
        silent_stepper_get_remaining_steps(&ss, &remainingSteps);

        if(remainingSteps == 0)
        {
            // Get magnetic flux density
            int16_t magnetic_flux_density = 0;
            int32_t magnetic_flux_density_sum = 0;
            size_t values = 10;
            for(size_t i = 0; i < values; i++)      // take a mean value
            {
                if(hall_effect_v2_get_magnetic_flux_density(&he, &magnetic_flux_density) < 0)
                {
                    std::cout << "Could not get Magnetic Flux Density, probably timeout\n";
                }
                magnetic_flux_density_sum += magnetic_flux_density;
            }
            nucmath::TableRow entry(std::vector<double>({(double)calibrationSteps, -(double)magnetic_flux_density_sum/values}));
            fluxStepsTable.addRow(entry);

            silent_stepper_set_steps(&ss, motorCalibStepsForValue);   // Drive steps forward
            calibrationSteps += motorCalibStepsForValue;
            rotationAngle += motorCalibStepsForValue*degreesPerStep;
        }
    }

    if(motorState == MotorState::rotLeft
            || motorState == MotorState::rotRight
            || motorState == MotorState::movingToZero)
    {
        int32_t remainingSteps = 0;
        silent_stepper_get_remaining_steps(&ss, &remainingSteps);

        if(remainingSteps == 0)
        {
            // Stop motor before disabling motor power
            silent_stepper_stop(&ss);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            silent_stepper_disable(&ss);

            ui->pB_rotate_left->setDisabled(false);
            ui->pB_rotate_right->setDisabled(false);

            if(motorState == MotorState::movingToZero)
            {
                rotationAngle = 0;
            }

            motorState = MotorState::stopped;
        }

        ui->l_remaining_steps->setText(nucmath::to_string(remainingSteps,2).c_str());
    }

    if(rotationAngle >= 360)
    {
        rotationAngle -= 360;
    }


}

void MainWindow::on_pB_connect_clicked()
{
    if(!connected)    // not connected -> connect
    {
        // Create IP connection
        ipcon_create(&ipcon);

        // Create device object
        hall_effect_v2_create(&he, UID_HE, &ipcon);
        silent_stepper_create(&ss, UID_SS, &ipcon);

        // Connect to brickd
        if(ipcon_connect(&ipcon, HOST, PORT) < 0)
        {
            std::cout << "Could not connect\n";
            return;
        }
        else
        {
            std::cout << "Connection established\n";
            connected = true;
            ui->pB_connect->setText("Disconnect");
        }
    }
    else
    {
        connected = false;
        ui->pB_connect->setText("Connect");

        // Stop motor before disabling motor power
        silent_stepper_stop(&ss);
        silent_stepper_set_speed_ramping(&ss, motorCalibAccel, motorCalibDeaccel);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        silent_stepper_disable(&ss);

        ipcon_disconnect(&ipcon);
        ipcon_destroy(&ipcon);
    }

}

void MainWindow::on_pB_rotate_left_clicked()
{
    int steps = ui->sB_rot_left_steps->value();

    // Don't use device before ipcon is connected
    if(!connected)
        return;

    if(motorState == MotorState::stopped)
    {
        silent_stepper_set_motor_current(&ss, motorMaxCurrent);
        silent_stepper_set_step_configuration(&ss, motorCalibStepResolution, true);
        silent_stepper_set_max_velocity(&ss, motorMaxVelocity);
        silent_stepper_set_speed_ramping(&ss, motorCalibAccel, motorCalibDeaccel);
        silent_stepper_enable(&ss);             // Enable motor power
        silent_stepper_set_steps(&ss, -steps);   // Drive steps forward

        ui->pB_rotate_left->setDisabled(true);
        ui->pB_rotate_right->setDisabled(true);

        motorState = MotorState::rotLeft;

        rotationAngle += steps*degreesPerStep;
    }
}

void MainWindow::on_pB_rotate_right_clicked()
{
    int steps = ui->sB_rot_right_steps->value();

    // Don't use device before ipcon is connected
    if(!connected)
        return;

    if(motorState == MotorState::stopped)
    {
        silent_stepper_set_motor_current(&ss, motorMaxCurrent);
        silent_stepper_set_step_configuration(&ss, motorCalibStepResolution, true);
        silent_stepper_set_max_velocity(&ss, motorMaxVelocity);
        silent_stepper_set_speed_ramping(&ss, motorCalibAccel, motorCalibDeaccel);
        silent_stepper_enable(&ss);             // Enable motor power
        silent_stepper_set_steps(&ss, steps);   // Drive steps backwards

        ui->pB_rotate_left->setDisabled(true);
        ui->pB_rotate_right->setDisabled(true);

        motorState = MotorState::rotRight;

        rotationAngle += steps*degreesPerStep;
    }
}

void MainWindow::on_pB_stop_rotation_clicked()
{
    ui->pB_rotate_left->setDisabled(false);
    ui->pB_rotate_right->setDisabled(false);

    if(!connected)
        return;

    // Stop motor before disabling motor power
    silent_stepper_stop(&ss);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    silent_stepper_disable(&ss);
}

void MainWindow::on_pB_rotate_for_calibration_clicked()
{
    if(!connected)
        return;

    if(motorState != MotorState::calibRot)
    {
        ui->pB_rotate_for_calibration->setText("Stop rotation");

        silent_stepper_set_motor_current(&ss, motorMaxCurrent);
        silent_stepper_set_step_configuration(&ss, motorCalibStepResolution, true);
        silent_stepper_set_max_velocity(&ss, motorMaxVelocity);
        silent_stepper_set_speed_ramping(&ss, motorCalibAccel, motorCalibDeaccel);
        silent_stepper_enable(&ss);     // Enable motor power

        motorState = MotorState::calibRot;
    }
    else
    {
        ui->pB_rotate_for_calibration->setText("Start rotation");

        // Stop motor before disabling motor power
        silent_stepper_stop(&ss);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        silent_stepper_disable(&ss);
    }
}

void MainWindow::on_sB_rot_left_degrees_valueChanged(double degrees)
{
    ui->sB_rot_left_steps->setValue(degrees/degreesPerStep);
}

void MainWindow::on_sB_rot_right_degrees_valueChanged(double degrees)
{
    ui->sB_rot_right_steps->setValue(degrees/degreesPerStep);
}

void MainWindow::on_sB_peak1pos_valueChanged(double peak1)
{
    startPositionSteps = peak1;

    double peak2 = ui->sB_peak2pos->value();
    double steps = peak2 - peak1;
    double steps_per_degree = steps/360.0;
    degreesPerStepFromCalibration = 360.0/steps;

    ui->l_steps_difference->setText(std::to_string(steps).c_str());
    ui->l_calib_steps_per_degree->setText(std::to_string(steps_per_degree).c_str());
    ui->l_calib_degrees_per_step->setText(nucmath::to_string(degreesPerStepFromCalibration, 7).c_str());
}

void MainWindow::on_sB_peak2pos_valueChanged(double peak2)
{
    double peak1 = ui->sB_peak1pos->value();
    double steps = peak2 - peak1;
    double steps_per_degree = steps/360.0;
    degreesPerStepFromCalibration = 360.0/steps;

    ui->l_steps_difference->setText(std::to_string(steps).c_str());
    ui->l_calib_steps_per_degree->setText(std::to_string(steps_per_degree).c_str());
    ui->l_calib_degrees_per_step->setText(nucmath::to_string(degreesPerStepFromCalibration, 7).c_str());
}

std::vector<double> MainWindow::findPeaks(const nucmath::DataTable& fluxStepsTable)
{
    std::vector<double> peakPos;

    if(fluxStepsTable.getNumberOfRows() > 10)
    {
        double miny = fluxStepsTable.getMin(1), maxy = fluxStepsTable.getMax(1);
        double thres = 0.5*(miny + maxy);
        double fwhm_start = 0, fwhm_end = 0;
        int counter = 0;
        size_t endi = fluxStepsTable.getNumberOfRows()-1;
        for(size_t i=0; i < (fluxStepsTable.getNumberOfRows()-1) && i < endi; ++i)
        {
            const auto& row = fluxStepsTable[i];
            const auto& row2 = fluxStepsTable[i+1];

            if(row[1] < thres && thres < row2[1] && counter == 0)
            {
                fwhm_start = row[0];
                counter++;
            }
            else if(row[1] > thres && thres > row2[1] && counter == 1)
            {
                fwhm_end = row[0];
                peakPos.push_back(0.5*(fwhm_start+fwhm_end));
                counter = 0;
            }
        }
    }

    return peakPos;
}

void MainWindow::on_pB_find_peaks_clicked()
{
    //fluxStepsTable.load("./magnetic_flux_vs_steps.txt");

    const auto& peaks = findPeaks(fluxStepsTable);
    double sigmaStart = 500;

    for(size_t i = 0; i < 2 && peaks.size() >= 2; i++)
    {
        double peakPos = peaks.at(peaks.size()-1-i);
        std::vector<double> xv;
        std::vector<double> yv;
        double miny = fluxStepsTable.getMin(1), maxy = fluxStepsTable.getMax(1);

        for(size_t j=0; j < (fluxStepsTable.getNumberOfRows()-1); ++j)
        {
            const auto& row = fluxStepsTable[j];

            if(peakPos-sigmaStart*5 < row[0] && row[0] < peakPos+sigmaStart*5)
            {
                xv.push_back(row[0]);
                yv.push_back(row[1]);
            }
        }

        std::vector<std::array<double, 3> > initial_p = { { { 10e6, 10e3, sigmaStart*1e5 },
                                                            { peakPos, peakPos-sigmaStart/2, peakPos+sigmaStart/2 },
                                                            { sigmaStart, sigmaStart*0.3, sigmaStart*5 },
                                                            { miny*1.0001, miny, 0.2*(miny+maxy) }} };
        std::vector<double> result;
        std::vector<double> result_sigma;

        nucmath::MODELFUNC fitModel = [&](const std::vector<double> &p, const nucmath::Vector<double> &x)
        {
            return nucmath::normal_dist_pdf({p[0],p[1],p[2]}, x(0)) + p[3];
        };

        nucmath::Minimizer minimizer;
        minimizer.setData(xv,yv);
        minimizer.setNumberOfSeedPoints(1);
        minimizer.setInitialPointsAndConstrains(initial_p);
        minimizer.setParameterNames({"A","mu","s", "b"});
        std::cout << minimizer.getFormatedInitialPoints();
        std::cout << minimizer.getFormatedConstrains();

        minimizer.setModelFunction(fitModel);
        minimizer.findFit(10000, 0.000001);
        result = minimizer.getResult();

        std::cout << minimizer.getFormatedFitResults() << std::endl;

        if(i == 0)
            ui->sB_peak2pos->setValue(result[1]);
        else if(i == 1)
            ui->sB_peak1pos->setValue(result[1]);

/*
        ui->labelFitOutput->setText(QString(" A=")+QString::number(result[0])
                +"  µ="+QString::number(result[1])
                +" ns   σ="+QString::number(result[2])
                +" ns   fwhm="+QString::number(result[2]*nucmath::SIGMA2FWHM) + " ns");
*/

        QVector<double> x, y;
        x.clear();
        y.clear();
        for(double xi=*xv.begin(); xi < *xv.rbegin(); xi+=100)
        {
            x.push_back(xi);
            double yi = fitModel(result, std::vector<double>({xi}));
            y.push_back(yi);
        }

        QCustomPlotZoom *customPlot = ui->plot_magnetic_flux_over_steps;
        int graphNr = 1+i;
        customPlot->graph(graphNr)->setData(x, y);
        customPlot->replot();
    }
}

void MainWindow::on_pB_apply_calibration_values_clicked()
{
    degreesPerStep = degreesPerStepFromCalibration;
}

void MainWindow::on_pB_move_to_start_position_clicked()
{
    // Don't use device before ipcon is connected
    if(!connected)
        return;

    if(motorState == MotorState::stopped
            && fluxStepsTable.getNumberOfRows() > 10)
    {
        const auto& row = *fluxStepsTable.getData().rbegin();
        const double peak1 = ui->sB_peak1pos->value();
        const double peak2 = ui->sB_peak2pos->value();
        const double steps_to_move = peak2 - peak1 - (row[0] - peak2);

        silent_stepper_set_motor_current(&ss, motorMaxCurrent);
        silent_stepper_set_step_configuration(&ss, motorCalibStepResolution, true);
        silent_stepper_set_max_velocity(&ss, motorMaxVelocity);
        silent_stepper_set_speed_ramping(&ss, motorCalibAccel, motorCalibDeaccel);
        silent_stepper_enable(&ss);                     // Enable motor power
        silent_stepper_set_steps(&ss, steps_to_move);   // Drive steps backwards

        ui->pB_rotate_left->setDisabled(true);
        ui->pB_rotate_right->setDisabled(true);

        motorState = MotorState::movingToZero;
    }
}

void MainWindow::on_pB_clear_graph_clicked()
{
    fluxStepsTable.getData().clear();
}

void MainWindow::on_pB_save_graphs_clicked()
{
    fluxStepsTable.save("./magnetic_flux_vs_steps.txt");
    timeSeries.save("./magnetic_flux_vs_time.txt");
}

