sam=loadDetectionData('~/srs_data/scan_deterministic_loop_2/scan_000001/scan_000001_detectreport.bin')
soc=loadDetectionData('~/Documents/Uhnder/Bitbucket/sabine-radar-sw/build-x86/socsim_000001_detectreport.bin')

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
plot(rad2deg(soc.az),rad2deg(soc.el),'ro',rad2deg(sam.az),rad2deg(sam.el),'bx');
            ylabel('Elevation [deg]');
            xlabel('Azimuth [deg]');
            title('Elevation vs Azimuth, SoC(red) and SamSim(blue)')
grid on;
