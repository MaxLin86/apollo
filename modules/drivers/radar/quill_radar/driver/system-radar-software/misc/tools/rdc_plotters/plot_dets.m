%sam=loadDetectionData('~/srs_data/scan_deterministic_loop_2/scan_000000/scan_000000_detectreport.bin')
%soc=loadDetectionData('~/Documents/Uhnder/Bitbucket/sabine-radar-sw/build-x86/socsim_000000_detectreport.bin')

function plot_dets(sam_detectreport,soc_detectreport)

    if ~exist('loadjson','file')
        addpath('../jsonlab/');
    end

    [fp1, fp2, fp3] = fileparts(sam_detectreport);
    samjsonfilepath = [fp1 '/' fp2(1:strfind(fp2,'_detectreport')) 'info.json'];
    scanData = loadjson(samjsonfilepath);
    sam_num_detections=scanData.num_detections;

    [fp1, fp2, fp3] = fileparts(soc_detectreport);
    socjsonfilepath = [fp1 '/' fp2(1:strfind(fp2,'_detectreport')) 'info.json'];
    scanData = loadjson(socjsonfilepath);
    soc_num_detections=scanData.num_detections;

    sam=loadDetectionData(sam_detectreport,sam_num_detections);
    soc=loadDetectionData(soc_detectreport,soc_num_detections);

    %plot only moving objs
    %sam.dopp = sam.dopp(abs(sam.dopp) > 1);
    %sam.range = sam.range(abs(sam.dopp) > 1);
    %sam.az = sam.az(abs(sam.dopp) > 1);

    figure, 
    subplot(1,3,1)
    plot(soc.dopp,soc.range,'ro',sam.dopp,sam.range,'bx');
                xlabel('Doppler [m/s]');
                ylabel('Range [m]');
                title('Doppler vs Range, SoC(red) and SamSim(blue)');            
    grid on;

    subplot(1,3,2)
    plot(rad2deg(soc.az),soc.range,'ro',rad2deg(sam.az),sam.range,'bx');
                xlabel('Azimuth [deg]');
                ylabel('Range [m]');
                title('Azimuth vs Range, SoC(red) and SamSim(blue)')
    grid on;

    subplot(1,3,3)
    plot(rad2deg(soc.az),soc.dopp,'ro',rad2deg(sam.az),sam.dopp,'bx');
                ylabel('Doppler [m/s]');
                xlabel('Azimuth [deg]');
                title('Azimuth va Doppler, SoC(red) and SamSim(blue)')
    grid on;
end
