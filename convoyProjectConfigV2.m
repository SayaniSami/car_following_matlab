function cfg = convoyProjectConfigV2()

cfg.dt = 0.1;
cfg.Tend = 60;
cfg.wheelbase = 2.8;

cfg.path.length = 260;
cfg.path.sampleStep = 0.5;
cfg.path.amplitude = 6;
cfg.path.freq1 = 0.025;
cfg.path.freq2 = 0.07;
cfg.path.phase2 = 0.8;

cfg.road.halfWidth = 4.5;

cfg.vehicle.length = 4.7;
cfg.vehicle.width = 1.9;
cfg.vehicle.minSpeed = 0;
cfg.vehicle.maxSpeed = 22;

cfg.leader.baseSpeed = 10;
cfg.leader.speedAmp1 = 1.2;
cfg.leader.speedAmp2 = 0.8;
cfg.leader.speedFreq1 = 0.03;
cfg.leader.speedFreq2 = 0.07;

cfg.follow.desiredGap = 12;
cfg.follow.lookAhead = 12;
cfg.follow.maxSteer = deg2rad(26);
cfg.follow.maxAccel = 3.8;
cfg.follow.maxBrake = 5.5;
cfg.follow.maxCruiseSpeed = 16.5;

cfg.follow.catchUpGain = 0.32;
cfg.follow.maxCatchUpBoost = 3.2;
cfg.follow.rejoinBoost = 2.2;
cfg.follow.rejoinTime = 2.8;

cfg.obstacles.count = 6;
cfg.obstacles.longRange = [40 220];
cfg.obstacles.radiusRange = [0.5 1.1];
cfg.obstacles.reactiveDistance = 20;
cfg.obstacles.avoidMargin = 1.5;

cfg.camera.imageWidth = 960;
cfg.camera.imageHeight = 540;
cfg.camera.fovDeg = 55;
cfg.camera.range = 80;
cfg.camera.targetWidth = 1.8;
cfg.camera.noiseStdPx = 2.0;
cfg.camera.mountX = 1.2;
cfg.camera.mountY = 0;

cfg.render.seed = 7;
cfg.render.makeVideo = false;
cfg.render.videoFile = 'convoy_demo.mp4';
cfg.render.figurePosition = [80 80 1500 800];

end