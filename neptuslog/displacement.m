function[n,e,d] = displacement(lat,lon,height,ref_lat,ref_lon,ref_height,x,y,z)
% Finds the offset from ref to lat,lon height with offset x,y,z
[temp_lat,temp_lon,temp_height] = ned2geodetic(x,y,z,lat,lon,height,wgs84Ellipsoid);
[n,e,d] = geodetic2ned(temp_lat,temp_lon,temp_height,ref_lat,ref_lon,ref_height,wgs84Ellipsoid);
end