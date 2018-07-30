if (toVisualize)
    %% prerequisite to visualize
    
    imageColorShow = getImgInTUMRGBDdataset(datasetPath, ICSLZEDdataset, cam, imgIdx, 'rgb');
    imageDepthShow = getImgInTUMRGBDdataset(datasetPath, ICSLZEDdataset, cam, imgIdx, 'depth');
    
    p_gc_DEMO = zeros(3, imgIdx);
    for k = 1:imgIdx
        p_gc_DEMO(:,k) = T_gc_DEMO{k}(1:3,4);
    end
    
    %% update color image part
    
    axes(ha1); cla;
    imshow(imageColorShow, []);  hold on;
    plot_tracked_point(ipRelations, cam); hold off;
    title('color image');
    
    %% update depth image part
    
    axes(ha2); cla;
    imshow(imageDepthShow, []); hold on;
    plot_tracked_point(ipRelations, cam); hold off;
    title('depth image');
    
    %% update 3D trajectory part
    
    axes(ha3); cla;
    % draw moving trajectory
    plot3(p_gc_DEMO(1,1:imgIdx), p_gc_DEMO(2,1:imgIdx), p_gc_DEMO(3,1:imgIdx), 'm', 'LineWidth', 2); hold on; grid on; axis equal;
    
    % draw camera body and frame
    plot_inertial_frame(0.5);
    RgcDEMO_current = T_gc_DEMO{imgIdx}(1:3,1:3);
    pgcDEMO_current = T_gc_DEMO{imgIdx}(1:3,4);
    plot_camera_frame(RgcDEMO_current, pgcDEMO_current, imageColorShow, 1.3, 'm'); hold off;
    refresh; pause(0.01);
    
    %% save current figure
    
    if (toSave)
        % save directory for MAT data
        SaveDir = [datasetPath '/ICRA2018'];
        if (~exist( SaveDir, 'dir' ))
            mkdir(SaveDir);
        end
        
        % save directory for images
        SaveImDir = [SaveDir '/DEMO'];
        if (~exist( SaveImDir, 'dir' ))
            mkdir(SaveImDir);
        end
        
        pause(0.1); refresh;
        saveImg = getframe(h);
        imwrite(saveImg.cdata , [SaveImDir sprintf('/%06d.png', imgIdx)]);
    end
end
