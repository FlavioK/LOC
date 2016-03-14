%Script for the experiment 1 of the localization course
%Author Age Kruijsen and Flavio Kreiliger
%Reset environment
clc;
clear all;

%Import all data
load '-ascii' '../data/20160313_184743_uVRU_LOWPASS.log'
data_LOWPASS  = load('-ascii','../data/20160313_184743_uVRU_LOWPASS.log');
data_STDEV  = importdata('../data/20160313_184743_uVRU_STDDEV.log');
data_NAV_HS = importdata('../data/20160313_184743_uVRU_NAV_HS.log','\t');

%Estimate the BIAS error
