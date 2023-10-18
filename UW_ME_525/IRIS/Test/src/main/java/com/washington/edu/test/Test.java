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
import edu.iris.dmc.extensions.entities.Trace;
// error handling
//<i>... more here...
//
// ServiceUtil is where we obtain all the "hooks" to the web services.
import edu.iris.dmc.service.ServiceUtil;
import edu.iris.dmc.extensions.fetch.TraceData;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.text.ParseException;
import java.util.Date;
import java.util.TimeZone;
// Import the error handling classes
// Import the classes specific to the STATION example
// Import the classes specific to the WAVEFORM example
// Import the classes specific to the EVENT example
// Import other Java classes>
//
public class Test {
  public static void main(String[] args) {
    // get access to the services
    // specify criteria
    // fetch data
    // process data
    TraceData traceFetcher = new TraceData();
    traceFetcher.setAppName("ALarson, APL-UW");
    
    // define the dates of interest
    DateFormat dfm = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS");
    dfm.setTimeZone(TimeZone.getTimeZone("GMT"));
    //
    try {
        Date startDate = dfm.parse("2023-01-10T00:00:00.000");
        Date endDate = dfm.parse("2023-01-10T00:01:45.000");
    
    Boolean includePZ = true;
    Trace traces []=null;  
    // Traces are unique in that you could provide a station criteria object (as in the station example) or
    // you can specify simple criteria directly in the call.</span>
    
    
    
    traces = traceFetcher.fetchTraces("IU", "AN*,B*","00","BHZ",
             startDate, endDate, 'B', includePZ);
    //loop through all the channels to display the details
    for (Trace trace : traces) {
       System.out.printf("Found %2s-%5s (%2s)  from %15s to %15s\n",
          trace.getNetwork(),trace.getStation(),trace.getChannel(),
          trace.getStartTime().toString(), trace.getEndTime().toString());
       System.out.printf("  This trace has %d samples, at %7.2f samples per second\n",
               trace.getSampleCount(), trace.getSampleRate());
       System.out.printf("  This channel is located at: %8.4f lat, %8.4f lon, elev %.0f m\n\n",
          trace.getLatitude(), trace.getLongitude(), trace.getElevation());
    }
    } catch (Exception pe) {
        System.out.println("Exception in Main");
    }
  }
}
