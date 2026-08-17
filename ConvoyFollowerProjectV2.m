classdef ConvoyFollowerProjectV2 < handle
    properties
        cfg
        pathS
        pathXY
        pathPsi
        pathKappa
        obstacles
        leader
        follower
        longState = struct('integral',0,'prevError',0,'derivFilt',0)
        steerState = struct('integral',0,'prevError',0,'derivFilt',0,'prevCmd',0)
        avoidState = struct('active',false,'obsIdx',0,'side',0)
        rejoinState = struct('active',false,'timer',0)
    end

    methods
        function obj = ConvoyFollowerProjectV2(cfg)
            obj.cfg = cfg;
            rng(cfg.render.seed);
            obj.buildPath();
            obj.spawnObstacles();
            obj.initialiseVehicles();
        end

        function buildPath(obj)
            s = 0:obj.cfg.path.sampleStep:obj.cfg.path.length;
            x = s;
            y = obj.cfg.path.amplitude*sin(obj.cfg.path.freq1*s) + ...
                4*sin(obj.cfg.path.freq2*s + obj.cfg.path.phase2);

            dx = gradient(x, obj.cfg.path.sampleStep);
            dy = gradient(y, obj.cfg.path.sampleStep);
            psi = atan2(dy, dx);

            dpsi = gradient(unwrap(psi), obj.cfg.path.sampleStep);
            speed = sqrt(dx.^2 + dy.^2);
            kappa = dpsi ./ max(speed, 1e-6);

            obj.pathS = s(:);
            obj.pathXY = [x(:) y(:)];
            obj.pathPsi = psi(:);
            obj.pathKappa = kappa(:);
        end

        function spawnObstacles(obj)
            n = obj.cfg.obstacles.count;
            sList = sort(obj.cfg.obstacles.longRange(1) + ...
                (obj.cfg.obstacles.longRange(2) - obj.cfg.obstacles.longRange(1)) * rand(n,1));

            rad = obj.cfg.obstacles.radiusRange(1) + ...
                diff(obj.cfg.obstacles.radiusRange) * rand(n,1);

            pos = zeros(n,2);

            for i = 1:n
                [x0,y0,psi0] = obj.pathAtS(sList(i));
                nvec = [-sin(psi0) cos(psi0)];

                usableHalf = obj.cfg.road.halfWidth - rad(i) - 0.55;
                usableHalf = max(0.7, usableHalf);
                lat = -usableHalf + 2*usableHalf*rand;

                pos(i,:) = [x0 y0] + lat*nvec;
            end

            obj.obstacles = struct( ...
                's', num2cell(sList), ...
                'x', num2cell(pos(:,1)), ...
                'y', num2cell(pos(:,2)), ...
                'r', num2cell(rad), ...
                'active', num2cell(true(n,1)));
        end

        function initialiseVehicles(obj)
            sL = 10;
            sF = max(0, sL - obj.cfg.follow.desiredGap);

            [xL,yL,psiL] = obj.pathAtS(sL);
            [xF,yF,psiF] = obj.pathAtS(sF);

            obj.leader = struct( ...
                's', sL, ...
                'x', xL, ...
                'y', yL, ...
                'yaw', psiL, ...
                'v', obj.cfg.leader.baseSpeed, ...
                'delta', 0);

            obj.follower = struct( ...
                's', sF, ...
                'x', xF, ...
                'y', yF, ...
                'yaw', psiF, ...
                'v', max(0, obj.cfg.leader.baseSpeed - 0.8), ...
                'delta', 0);
        end

        function results = run(obj)
            cfg = obj.cfg;
            N = floor(cfg.Tend / cfg.dt) + 1;
            t = (0:N-1)' * cfg.dt;

            leaderState = zeros(N,4);
            followerState = zeros(N,4);
            pathError = zeros(N,1);
            headingError = zeros(N,1);
            steerCmdLog = zeros(N,1);
            accelCmdLog = zeros(N,1);
            obstacleClearance = inf(N,1);
            chosenObs = zeros(N,1);

            fig = figure('Color','w','Position',cfg.render.figurePosition);

            if cfg.render.makeVideo
                vw = VideoWriter(cfg.render.videoFile,'MPEG-4');
                vw.FrameRate = round(1 / cfg.dt);
                open(vw);
            else
                vw = [];
            end

            for k = 1:N
                tk = t(k);

                obj.leader = obj.updateLeader(tk);
                detection = obj.syntheticDetection();
                [accCmd, steerCmd, aux] = obj.followerController(detection);
                obj.follower = obj.stepVehicle(obj.follower, accCmd, steerCmd);

                leaderState(k,:) = [obj.leader.x obj.leader.y obj.leader.yaw obj.leader.v];
                followerState(k,:) = [obj.follower.x obj.follower.y obj.follower.yaw obj.follower.v];
                pathError(k) = aux.pathError;
                headingError(k) = aux.headingError;
                steerCmdLog(k) = steerCmd;
                accelCmdLog(k) = accCmd;
                obstacleClearance(k) = aux.clearance;
                chosenObs(k) = aux.chosenObstacle;

                obj.renderFrame(fig, tk, detection, aux, ...
                    leaderState(1:k,:), followerState(1:k,:), ...
                    pathError(1:k), headingError(1:k), ...
                    steerCmdLog(1:k), accelCmdLog(1:k), obstacleClearance(1:k));

                drawnow limitrate

                if ~isempty(vw)
                    writeVideo(vw, getframe(fig));
                end
            end

            if ~isempty(vw)
                close(vw);
            end

            results.time = t;
            results.leaderState = leaderState;
            results.followerState = followerState;
            results.pathError = pathError;
            results.headingError = headingError;
            results.steerCmd = steerCmdLog;
            results.accelCmd = accelCmdLog;
            results.obstacleClearance = obstacleClearance;
            results.chosenObstacle = chosenObs;
        end

        function leader = updateLeader(obj, t)
            vRef = obj.cfg.leader.baseSpeed + ...
                   obj.cfg.leader.speedAmp1*sin(2*pi*obj.cfg.leader.speedFreq1*t) + ...
                   obj.cfg.leader.speedAmp2*sin(2*pi*obj.cfg.leader.speedFreq2*t + 0.9);

            acc = 0.7 * (vRef - obj.leader.v);
            obj.leader.v = max(obj.cfg.vehicle.minSpeed, ...
                               min(obj.cfg.vehicle.maxSpeed, obj.leader.v + acc*obj.cfg.dt));

            sNext = min(obj.cfg.path.length, obj.leader.s + obj.leader.v*obj.cfg.dt);
            [x, y, psi] = obj.pathAtS(sNext);

            obj.leader.s = sNext;
            obj.leader.x = x;
            obj.leader.y = y;
            obj.leader.yaw = psi;
            leader = obj.leader;
        end

        function detection = syntheticDetection(obj)
            cfg = obj.cfg;
            cam = obj.cameraPose();

            dx = obj.leader.x - cam(1);
            dy = obj.leader.y - cam(2);

            R = [cos(cam(3)) sin(cam(3)); -sin(cam(3)) cos(cam(3))];
            rel = R * [dx; dy];
            xCam = rel(1);
            yCam = rel(2);

            fx = (cfg.camera.imageWidth/2) / tan(deg2rad(cfg.camera.fovDeg/2));
            valid = xCam > 2 && xCam < cfg.camera.range;

            if valid
                cx = cfg.camera.imageWidth/2 - fx*(yCam/xCam) + cfg.camera.noiseStdPx*randn;
                bw = fx*cfg.camera.targetWidth/max(xCam,1) + cfg.camera.noiseStdPx*randn;
                bh = 1.7*bw;

                x1 = cx - bw/2;
                x2 = cx + bw/2;
                y2 = 0.78*cfg.camera.imageHeight;
                y1 = y2 - bh;

                valid = x1 >= 1 && x2 <= cfg.camera.imageWidth && y1 >= 1 && bw > 12;
            else
                cx = cfg.camera.imageWidth/2;
                bw = 0;
                x1 = cx;
                x2 = cx;
                y1 = cfg.camera.imageHeight/2;
                y2 = y1;
            end

            if valid
                estRange = fx*cfg.camera.targetWidth/max(bw,1);
                lateral = -(cx - cfg.camera.imageWidth/2) * estRange / fx;
            else
                estRange = hypot(dx,dy);
                lateral = yCam;
            end

            detection = struct( ...
                'valid', valid, ...
                'estimatedRange', estRange, ...
                'lateralEstimate', lateral, ...
                'bbox', [x1 y1 x2 y2], ...
                'pixelCenter', cx, ...
                'rawForward', xCam, ...
                'rawLateral', yCam);
        end

        function [accCmd, steerCmd, aux] = followerController(obj, detection)
            cfg = obj.cfg;

            sNear = obj.nearestS(obj.follower.x, obj.follower.y);
            lookS = min(sNear + cfg.follow.lookAhead, cfg.path.length);
            [xPath, yPath, ~] = obj.pathAtS(lookS);
            [~, ~, psiRef] = obj.pathAtS(sNear);

            pathError = obj.crossTrackError(obj.follower.x, obj.follower.y, sNear);
            headingError = localWrapToPi(psiRef - obj.follower.yaw);

            trueGap = max(0, obj.leader.s - obj.follower.s);
            if detection.valid
                rangeGap = detection.estimatedRange;
            else
                rangeGap = trueGap;
            end

            targetSpeed = obj.leader.v;

            if rangeGap < cfg.follow.desiredGap + 1.0
                targetSpeed = min(targetSpeed, obj.leader.v - 0.5);
            end

            xq = xPath;
            yq = yPath;
            clearObs = inf;
            chosenObs = 0;
            avoidActive = false;

            bestMetric = inf;
            candidateIdx = 0;
            candidateSide = 0;

            for i = 1:numel(obj.obstacles)
                if ~obj.obstacles(i).active
                    continue;
                end

                ox = obj.obstacles(i).x;
                oy = obj.obstacles(i).y;
                sObs = obj.nearestS(ox, oy);
                ds = sObs - sNear;

                if ds < -6 || ds > cfg.obstacles.reactiveDistance
                    continue;
                end

                dCenter = hypot(obj.follower.x - ox, obj.follower.y - oy);
                dEdge = dCenter - obj.obstacles(i).r;

                if dEdge < clearObs
                    clearObs = dEdge;
                    chosenObs = i;
                end

                metric = ds + 0.15*dEdge;
                if metric < bestMetric
                    [xo, yo, psio] = obj.pathAtS(sObs);
                    nvec = [-sin(psio) cos(psio)];
                    obsLat = dot([ox - xo, oy - yo], nvec);

                    if obsLat >= 0
                        side = -1;
                    else
                        side = 1;
                    end

                    bestMetric = metric;
                    candidateIdx = i;
                    candidateSide = side;
                end
            end

            wasAvoiding = obj.avoidState.active;

            if obj.avoidState.active
                idx = obj.avoidState.obsIdx;
                if idx < 1 || idx > numel(obj.obstacles) || ~obj.obstacles(idx).active
                    obj.avoidState.active = false;
                    obj.avoidState.obsIdx = 0;
                    obj.avoidState.side = 0;
                else
                    sObs = obj.nearestS(obj.obstacles(idx).x, obj.obstacles(idx).y);
                    dsHold = sObs - sNear;

                    if dsHold < -8
                        obj.avoidState.active = false;
                        obj.avoidState.obsIdx = 0;
                        obj.avoidState.side = 0;
                    end
                end
            end

            if wasAvoiding && ~obj.avoidState.active
                obj.rejoinState.active = true;
                obj.rejoinState.timer = cfg.follow.rejoinTime;
            end

            if ~obj.avoidState.active && candidateIdx > 0
                obj.avoidState.active = true;
                obj.avoidState.obsIdx = candidateIdx;
                obj.avoidState.side = candidateSide;
                obj.rejoinState.active = false;
                obj.rejoinState.timer = 0;
            end

            if obj.avoidState.active
                idx = obj.avoidState.obsIdx;
                side = obj.avoidState.side;

                ox = obj.obstacles(idx).x;
                oy = obj.obstacles(idx).y;
                rr = obj.obstacles(idx).r + cfg.obstacles.avoidMargin + 1.1;

                sObs = obj.nearestS(ox, oy);
                ds = sObs - sNear;

                [xo, yo, psio] = obj.pathAtS(sObs);
                nvec = [-sin(psio) cos(psio)];

                if ds > -8 && ds < cfg.obstacles.reactiveDistance
                    avoidActive = true;

                    tangentOffset = side * min(obj.cfg.road.halfWidth - 0.55, rr + 1.1);
                    aimS = min(sObs + 5.0, cfg.path.length);
                    [xa, ya, ~] = obj.pathAtS(aimS);

                    xq = xa + tangentOffset*nvec(1);
                    yq = ya + tangentOffset*nvec(2);

                    vx = xq - obj.follower.x;
                    vy = yq - obj.follower.y;

                    distLine = abs((oy - obj.follower.y)*vx - (ox - obj.follower.x)*vy) / max(hypot(vx,vy), 1e-6);

                    if distLine < rr
                        extra = (rr - distLine) + 1.2;
                        xq = xq + side*extra*nvec(1);
                        yq = yq + side*extra*nvec(2);
                    end

                    dToObs = hypot(obj.follower.x - ox, obj.follower.y - oy);
                    if dToObs < rr + 3
                        targetSpeed = min(targetSpeed, 3.8);
                    else
                        targetSpeed = min(targetSpeed, 5.2);
                    end
                end
            end

            if obj.rejoinState.active
                obj.rejoinState.timer = max(0, obj.rejoinState.timer - cfg.dt);
                if obj.rejoinState.timer <= 0
                    obj.rejoinState.active = false;
                end
            end

            gapError = rangeGap - cfg.follow.desiredGap;

            if detection.valid && gapError > 0
                catchBoost = min(cfg.follow.maxCatchUpBoost, cfg.follow.catchUpGain * gapError);
                targetSpeed = max(targetSpeed, obj.leader.v + catchBoost);
            end

            if obj.rejoinState.active && gapError > 1.5
                targetSpeed = max(targetSpeed, obj.leader.v + cfg.follow.rejoinBoost);
            end

            targetSpeed = min(targetSpeed, cfg.follow.maxCruiseSpeed);

            alpha = localWrapToPi(atan2(yq - obj.follower.y, xq - obj.follower.x) - obj.follower.yaw);

            if avoidActive
                ppLook = max(7, cfg.follow.lookAhead - 2);
                steerPP = atan2(2 * cfg.wheelbase * sin(alpha), ppLook);
                steerSignal = 1.25 * steerPP + 0.03 * pathError + 0.08 * headingError;
            else
                ppLook = max(8, cfg.follow.lookAhead);
                steerPP = atan2(2 * cfg.wheelbase * sin(alpha), ppLook);
                steerSignal = 0.90 * steerPP + 0.07 * pathError + 0.15 * headingError;
            end

            obj.steerState.integral = obj.steerState.integral + steerSignal * cfg.dt;
            obj.steerState.integral = max(-0.35, min(0.35, obj.steerState.integral));

            rawDeriv = (steerSignal - obj.steerState.prevError) / cfg.dt;
            obj.steerState.derivFilt = 0.90 * obj.steerState.derivFilt + 0.10 * rawDeriv;
            obj.steerState.prevError = steerSignal;

            rawSteer = 0.82 * steerSignal + 0.008 * obj.steerState.integral + 0.018 * obj.steerState.derivFilt;
            rawSteer = max(-cfg.follow.maxSteer, min(cfg.follow.maxSteer, rawSteer));

            maxRate = deg2rad(55);
            steerCmd = obj.steerState.prevCmd + ...
                max(-maxRate*cfg.dt, min(maxRate*cfg.dt, rawSteer - obj.steerState.prevCmd));
            obj.steerState.prevCmd = steerCmd;

            speedErr = targetSpeed - obj.follower.v;

            obj.longState.integral = obj.longState.integral + speedErr*cfg.dt;
            obj.longState.integral = max(-5, min(5, obj.longState.integral));

            speedDeriv = (speedErr - obj.longState.prevError) / cfg.dt;
            obj.longState.derivFilt = 0.88 * obj.longState.derivFilt + 0.12 * speedDeriv;
            obj.longState.prevError = speedErr;

            accCmd = 0.78 * speedErr + 0.045 * obj.longState.integral + 0.012 * obj.longState.derivFilt;

            if avoidActive && clearObs < 6
                accCmd = min(accCmd, -1.8);
            end

            accCmd = max(-cfg.follow.maxBrake, min(cfg.follow.maxAccel, accCmd));

            aux = struct( ...
                'pathError', pathError, ...
                'headingError', headingError, ...
                'clearance', clearObs, ...
                'chosenObstacle', chosenObs, ...
                'steerDirection', obj.steerDirectionText(steerCmd), ...
                'targetSpeed', targetSpeed, ...
                'gapError', gapError, ...
                'rejoinActive', obj.rejoinState.active);
        end

        function vehicle = stepVehicle(obj, vehicle, accel, steer)
            dt = obj.cfg.dt;

            vehicle.delta = max(-obj.cfg.follow.maxSteer, min(obj.cfg.follow.maxSteer, steer));
            vehicle.v = max(obj.cfg.vehicle.minSpeed, ...
                            min(obj.cfg.vehicle.maxSpeed, vehicle.v + accel*dt));

            vehicle.x = vehicle.x + vehicle.v*cos(vehicle.yaw)*dt;
            vehicle.y = vehicle.y + vehicle.v*sin(vehicle.yaw)*dt;
            vehicle.yaw = localWrapToPi(vehicle.yaw + ...
                (vehicle.v / obj.cfg.wheelbase) * tan(vehicle.delta) * dt);

            vehicle.s = obj.nearestS(vehicle.x, vehicle.y);
        end

        function pose = cameraPose(obj)
            R = [cos(obj.follower.yaw) -sin(obj.follower.yaw); ...
                 sin(obj.follower.yaw)  cos(obj.follower.yaw)];
            off = R * [obj.cfg.camera.mountX; obj.cfg.camera.mountY];
            pose = [obj.follower.x + off(1), obj.follower.y + off(2), obj.follower.yaw];
        end

        function [x,y,psi] = pathAtS(obj,s)
            s = max(min(obj.pathS), min(max(obj.pathS), s));
            x = interp1(obj.pathS, obj.pathXY(:,1), s, 'linear');
            y = interp1(obj.pathS, obj.pathXY(:,2), s, 'linear');
            psi = interp1(obj.pathS, obj.pathPsi, s, 'linear');
        end

        function s = nearestS(obj, x, y)
            d = hypot(obj.pathXY(:,1)-x, obj.pathXY(:,2)-y);
            [~, idx] = min(d);
            s = obj.pathS(idx);
        end

        function err = crossTrackError(obj, x, y, s)
            [xp, yp, psi] = obj.pathAtS(s);
            nvec = [-sin(psi) cos(psi)];
            err = dot([x-xp, y-yp], nvec);
        end

        function txt = steerDirectionText(~, steer)
            if steer > deg2rad(1.2)
                txt = 'Turning Left';
            elseif steer < -deg2rad(1.2)
                txt = 'Turning Right';
            else
                txt = 'Nearly Straight';
            end
        end

        function renderFrame(obj, fig, t, detection, aux, leaderHist, followerHist, pathHist, headingHist, steerHist, accelHist, clearHist)
            cfg = obj.cfg;

            pathHist = pathHist(:);
            headingHist = headingHist(:);
            steerHist = steerHist(:);
            accelHist = accelHist(:);
            clearHist = clearHist(:);

            n = numel(pathHist);
            tt = (0:n-1)' * cfg.dt;

            clf(fig);
            tiledlayout(fig,2,3,'Padding','compact','TileSpacing','compact');

            nexttile(1,[2 1]);
            obj.renderRoadScene();
            title(sprintf('Path scene   t = %.1f s', t));

            nexttile(2);
            obj.renderCameraView(detection, aux);
            title('Follower camera + detection');

            nexttile(3);
            obj.renderBirdsEye(detection, aux);
            title('Bird''s-eye obstacle avoidance');

            nexttile(5);
            hold on; grid on;
            plot(tt, pathHist, 'b', 'LineWidth', 1.6);
            yline(0, 'k--');
            xlim([0 max(cfg.dt, tt(end))]);
            ylim([-6 10]);
            xlabel('Time [s]');
            ylabel('Path error [m]');
            title('Cross-track error');

            nexttile(6);
            yyaxis left
            hold on; grid on;
            plot(tt, rad2deg(steerHist), 'g', 'LineWidth', 1.4);
            plot(tt, rad2deg(headingHist), 'm', 'LineWidth', 1.3);
            ylabel('Angle [deg]');
            ylim([-45 45]);

            yyaxis right
            plot(tt, leaderHist(:,4), 'r--', 'LineWidth', 1.2);
            plot(tt, followerHist(:,4), 'b-.', 'LineWidth', 1.2);
            plot(tt, accelHist, 'c:', 'LineWidth', 1.1);
            plot(tt, clearHist, 'k:', 'LineWidth', 1.2);
            xlim([0 max(cfg.dt, tt(end))]);
            ylabel('Speed / accel / clearance');
            xlabel('Time [s]');
            title('Steering, speed, clearance');
            legend('Steer cmd','Heading error','Leader speed','Follower speed','Accel cmd','Obstacle clearance','Location','best');
        end

        function renderRoadScene(obj)
            hold on; axis equal; grid on;

            sMid = 0.5*(obj.follower.s + obj.leader.s);
            sMin = max(0, sMid - 28);
            sMax = min(obj.cfg.path.length, sMid + 28);

            sPlot = linspace(sMin, sMax, 500);
            cxy = zeros(numel(sPlot),2);
            up = zeros(numel(sPlot),2);
            lo = zeros(numel(sPlot),2);

            for i = 1:numel(sPlot)
                [cxy(i,1), cxy(i,2), psi] = obj.pathAtS(sPlot(i));
                nvec = [-sin(psi) cos(psi)];
                up(i,:) = cxy(i,:) + obj.cfg.road.halfWidth*nvec;
                lo(i,:) = cxy(i,:) - obj.cfg.road.halfWidth*nvec;
            end

            patch([up(:,1); flipud(lo(:,1))], [up(:,2); flipud(lo(:,2))], ...
                [0.18 0.18 0.18], 'EdgeColor','none');

            plot(cxy(:,1), cxy(:,2), 'y--', 'LineWidth', 1.2);

            for sMark = max(0,floor(sMin/20)*20):20:floor(sMax/20)*20
                [xm, ym, psim] = obj.pathAtS(sMark);
                nvec = [-sin(psim) cos(psim)];
                plot([xm-3*nvec(1) xm+3*nvec(1)], [ym-3*nvec(2) ym+3*nvec(2)], 'w-', 'LineWidth', 1.0);
            end

            for i = 1:numel(obj.obstacles)
                if obj.obstacles(i).active
                    viscircles([obj.obstacles(i).x obj.obstacles(i).y], obj.obstacles(i).r, ...
                        'Color',[1 0.5 0], 'LineWidth', 1.2);
                    viscircles([obj.obstacles(i).x obj.obstacles(i).y], obj.obstacles(i).r + obj.cfg.obstacles.avoidMargin + 1.1, ...
                        'Color',[1 0.8 0.2], 'LineStyle',':', 'LineWidth', 0.8);
                end
            end

            obj.drawCar(obj.leader, [0.86 0.18 0.18]);
            obj.drawCar(obj.follower, [0.12 0.35 0.92]);

            quiver(obj.follower.x, obj.follower.y, ...
                4.5*cos(obj.follower.yaw), 4.5*sin(obj.follower.yaw), 0, ...
                'c', 'LineWidth', 2, 'MaxHeadSize', 1.2);

            xlim([min([up(:,1); lo(:,1)])-2, max([up(:,1); lo(:,1)])+2]);
            ylim([min([up(:,2); lo(:,2)])-4, max([up(:,2); lo(:,2)])+4]);
            xlabel('X [m]');
            ylabel('Y [m]');
        end

        function renderCameraView(obj, detection, aux)
            img = uint8(ones(obj.cfg.camera.imageHeight, obj.cfg.camera.imageWidth, 3) * 78);
            horizon = round(0.42*obj.cfg.camera.imageHeight);

            sky = reshape(uint8([170 205 235]),1,1,3);
            road = reshape(uint8([55 55 55]),1,1,3);
            img(1:horizon,:,:) = repmat(sky, horizon, obj.cfg.camera.imageWidth);
            img(horizon+1:end,:,:) = repmat(road, obj.cfg.camera.imageHeight-horizon, obj.cfg.camera.imageWidth);

            laneMid = obj.cfg.camera.imageWidth/2;
            for k = 0:10
                y1 = round(horizon + k*45);
                y2 = min(obj.cfg.camera.imageHeight, y1+25);
                xoff = round(0.12*(y1-horizon));
                xLeft = laneMid - 180 - xoff;
                xRight = laneMid + 180 + xoff;

                if y1 < obj.cfg.camera.imageHeight
                    img(y1:y2,max(1,xLeft-4):min(obj.cfg.camera.imageWidth,xLeft+4),:) = 255;
                    img(y1:y2,max(1,xRight-4):min(obj.cfg.camera.imageWidth,xRight+4),:) = 255;
                end
            end

            if detection.valid
                bb = round(detection.bbox);
                w = max(24, bb(3)-bb(1));
                h = max(42, bb(4)-bb(2));
                x = max(1, min(obj.cfg.camera.imageWidth-w, bb(1)));
                y = max(1, min(obj.cfg.camera.imageHeight-h, bb(2)));

                img = insertShape(img,'FilledRectangle',[x y w h], ...
                    'Color',[220 60 60],'Opacity',0.75);
                img = insertShape(img,'Rectangle',[x y w h], ...
                    'Color','green','LineWidth',4);

                img = insertText(img,[x max(1,y-28)], sprintf('Leader  %.1f m',detection.estimatedRange), ...
                    'FontSize',22,'BoxColor','green','TextColor','black');

                cx = round(detection.pixelCenter);
                img = insertShape(img,'Line',[obj.cfg.camera.imageWidth/2 40 cx obj.cfg.camera.imageHeight-80], ...
                    'Color','yellow','LineWidth',3);
            end

            img = insertText(img,[20 20], sprintf('Detection: %d',detection.valid), ...
                'FontSize',22,'BoxColor','blue');
            img = insertText(img,[20 62], sprintf('Steer: %.1f deg',rad2deg(obj.follower.delta)), ...
                'FontSize',22,'BoxColor','cyan');
            img = insertText(img,[20 104], sprintf('Gap err: %.2f m',aux.gapError), ...
                'FontSize',22,'BoxColor','magenta');
            img = insertText(img,[20 146], sprintf('Target v: %.2f m/s',aux.targetSpeed), ...
                'FontSize',22,'BoxColor','yellow','TextColor','black');
            img = insertText(img,[20 188], sprintf('Rejoin: %d',aux.rejoinActive), ...
                'FontSize',22,'BoxColor','green');

            image(img);
            axis image off;
        end

        function renderBirdsEye(obj, detection, aux)
            hold on; axis equal; grid on;

            rectangle('Position',[-6 0 12 70], ...
                'FaceColor',[0.94 0.94 0.94], ...
                'EdgeColor',[0.35 0.35 0.35]);

            plot([0 0],[0 65],'y--','LineWidth',1.2);
            patch([-0.95 0.95 0.95 -0.95],[0 0 4.7 4.7], ...
                [0.12 0.35 0.92],'FaceAlpha',0.75,'EdgeColor','none');

            if detection.rawForward > 0
                lat = detection.rawLateral;
                fwd = min(max(detection.rawForward, 0), 65);

                patch([-0.95 0.95 0.95 -0.95] + lat, ...
                    [fwd-2.3 fwd-2.3 fwd+2.3 fwd+2.3], ...
                    [0.86 0.18 0.18],'FaceAlpha',0.35,'EdgeColor','none');
            end

            for i = 1:numel(obj.obstacles)
                if obj.obstacles(i).active
                    rel = obj.worldToFollowerFrame(obj.obstacles(i).x, obj.obstacles(i).y);
                    if rel(2) > -5 && rel(2) < 70
                        scatter(rel(1), rel(2), 90 + 20*obj.obstacles(i).r, ...
                            'filled', 'MarkerFaceColor', [1 0.5 0], 'MarkerEdgeColor', [0.4 0.2 0]);
                    end
                end
            end

            quiver(0, 4.7, 9*sin(obj.follower.delta), 9*cos(obj.follower.delta), 0, ...
                'c', 'LineWidth', 2.5, 'MaxHeadSize', 1.2);

            text(-5.5, 66, sprintf('Steer = %.1f deg', rad2deg(obj.follower.delta)), ...
                'FontSize', 11, 'FontWeight','bold');
            text(-5.5, 62, aux.steerDirection, 'FontSize', 11);
            text(-5.5, 58, sprintf('Gap err = %.2f m', aux.gapError), 'FontSize', 11);
            text(-5.5, 54, sprintf('Target v = %.2f', aux.targetSpeed), 'FontSize', 11);

            xlim([-10 10]);
            ylim([0 70]);
            xlabel('Lateral [m]');
            ylabel('Forward [m]');
        end

        function rel = worldToFollowerFrame(obj, xw, yw)
            dx = xw - obj.follower.x;
            dy = yw - obj.follower.y;

            R = [cos(obj.follower.yaw) sin(obj.follower.yaw); ...
                -sin(obj.follower.yaw) cos(obj.follower.yaw)];

            p = R * [dx; dy];
            rel = [p(2), p(1) + 4.7];
        end

        function drawCar(obj, vehicle, color)
            L = obj.cfg.vehicle.length;
            W = obj.cfg.vehicle.width;

            body = [ L/2  W/2;
                     L/2 -W/2;
                    -L/2 -W/2;
                    -L/2  W/2 ]';

            R = [cos(vehicle.yaw) -sin(vehicle.yaw); ...
                 sin(vehicle.yaw)  cos(vehicle.yaw)];

            worldPts = R*body + [vehicle.x; vehicle.y];

            patch(worldPts(1,:), worldPts(2,:), color, ...
                'FaceAlpha', 0.88, 'EdgeColor', [0 0 0], 'LineWidth', 1.2);

            nose = R*[L/2;0] + [vehicle.x;vehicle.y];
            quiver(vehicle.x, vehicle.y, nose(1)-vehicle.x, nose(2)-vehicle.y, 0, ...
                'k', 'LineWidth', 1.4, 'MaxHeadSize', 1);

            wheelLen = 0.65;
            wheelOffX = 1.25;
            wheelOffY = 0.82;

            centers = [ wheelOffX  wheelOffY;
                        wheelOffX -wheelOffY;
                       -wheelOffX  wheelOffY;
                       -wheelOffX -wheelOffY ]';

            for i = 1:4
                c = R*centers(:,i) + [vehicle.x; vehicle.y];
                ang = vehicle.yaw + (i <= 2)*vehicle.delta;
                p1 = c + 0.5*wheelLen*[cos(ang); sin(ang)];
                p2 = c - 0.5*wheelLen*[cos(ang); sin(ang)];
                plot([p1(1) p2(1)], [p1(2) p2(2)], 'k', 'LineWidth', 3);
            end
        end
    end
end

function a = localWrapToPi(a)
a = mod(a + pi, 2*pi) - pi;
end