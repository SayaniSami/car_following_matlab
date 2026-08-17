clear;
clc;
close all;
clear classes;
rehash;

cfg = convoyProjectConfigV2();
project = ConvoyFollowerProjectV2(cfg);
results = project.run();