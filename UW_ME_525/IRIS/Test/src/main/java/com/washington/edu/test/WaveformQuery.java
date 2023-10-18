/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */

package com.washington.edu.test;

/**
 *
 * @author Arnie Larson alarson@apl.washington.edu
 * 
 */
// Import the error handling classes
import edu.iris.dmc.criteria.CriteriaException;  
import edu.iris.dmc.criteria.WaveformCriteria;
import edu.iris.dmc.extensions.entities.Trace;
// error handling
//<i>... more here...
//
// ServiceUtil is where we obtain all the "hooks" to the web services.
import edu.iris.dmc.service.ServiceUtil;
import edu.iris.dmc.extensions.fetch.TraceData;
import edu.iris.dmc.fdsn.station.model.Channel;
import edu.iris.dmc.service.WaveformService;
import edu.iris.dmc.timeseries.model.Segment;
import edu.iris.dmc.timeseries.model.Timeseries;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.text.ParseException;
import java.util.Date;
import java.util.List;
import java.util.TimeZone;
// Import the error handling classes
// Import the classes specific to the STATION example
// Import the classes specific to the WAVEFORM example
// Import the classes specific to the EVENT example
// Import other Java classes>
//
public class WaveformQuery {
  public static void main(String[] args) {
    // get access to the services
    // specify criteria
    // fetch data
    // process data
    try {
        ServiceUtil serviceUtil = ServiceUtil.getInstance();
        serviceUtil.setAppName("ALarson, APL-UW");
        WaveformService waveformService = serviceUtil.getWaveformService();


        DateFormat dfm = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss");
        dfm.setTimeZone(TimeZone.getTimeZone("UTC"));
        Date startDate = dfm.parse("2023-01-11T00:00:00");
        Date endDate = dfm.parse("2023-01-11T00:01:00");
        String NETWORK = "IM";
        String STATION = "H11N1";
        String LOCATION = "*";
        String CHANNEL = "EDH";
        WaveformCriteria criteria = new WaveformCriteria();
        criteria.add(NETWORK, STATION, LOCATION, CHANNEL, startDate, endDate);
        
        List<Timeseries> timeSeriesCollection = waveformService.fetch(criteria);
        //loop through all the channels to display the details
        for(Timeseries timeseries:timeSeriesCollection){
            System.out.println(timeseries.getNetworkCode() + "-" +
            timeseries.getStationCode() + " (" + timeseries.getChannelCode() + "), loc:" +
               timeseries.getLocation());
               for(Segment segment:timeseries.getSegments()){
                  System.out.printf("Segment:\n");
                  System.out.printf("\tStart: %s", segment.getStartTime());
                  System.out.printf("  %d samples exist in this segment",
                     segment.getSampleCount());         
                  System.out.printf("\tData Type: %s",segment.getType());
            }
         }
    } catch (Exception pe) {
        System.out.printf("Exception in main: %s\n", pe.toString());
    }
  }
}
