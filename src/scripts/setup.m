clear;
close all;
set(0,'defaulttextInterpreter','latex');

addpath('../models')
addpath('../functions')
addpath('../generatedCache')
addpath('../../data')
addpath('../core')

cfg = Simulink.fileGenControl('getConfig');
cfg.CacheFolder = fullfile(eval(['pwd']),'../generatedCache');
Simulink.fileGenControl('setConfig', 'config', cfg,'createDir',true);

addpath('../generatedCache')
