#pragma once

#include "StdInclude.h"
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <filesystem>

class FileHandler{
public:
    FileHandler();

    void ReadModelConfigFile(const std::string& yamlFilePath, task &_taskConfig);

    void ReadSettingsFile(const std::string& settingsFilePath);

    void SaveTrajecInformation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices,
                               std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string file_prefix);

    void SaveTaskToFile(std::string file_prefix, int file_num, const stateVectorList &state_vector, const vector<residual> &residuals);
    void LoadTaskFromFile(std::string task_prefix, int file_num, stateVectorList &state_vector, vector<residual> &residuals);

    std::string project_run_mode;
    std::string taskName;
    std::string optimiser;
    std::string taskInitMode;
    int csvRow = 0;
    bool async_mpc = true;
    bool record_trajectory = false;
    ofstream fileOutput;

    int minIter;
    int maxIter;

private:
    std::string projectParentPath;

};