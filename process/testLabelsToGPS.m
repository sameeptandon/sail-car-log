load('gps_coord.mat')
figure; hold on; 

plot(l(:,2), l(:,1), 'r.');
plot(m(:,2), m(:,1), 'b.');
plot(r(:,2), r(:,1), 'g.'); 

plot_google_map('maptype', 'satellite', 'APIKey', 'AIzaSyDF0fWrlInoYoypZ0y10calIz50zK62OKI');