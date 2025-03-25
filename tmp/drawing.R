library(googleVis)

latlong<-readLines("output.txt")
df<-data.frame(latlong)
G<-gvisGeoChart(df, locationvar="latlong")
plot(G)
