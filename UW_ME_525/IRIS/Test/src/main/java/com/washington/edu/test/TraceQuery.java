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
import edu.iris.dmc.fdsn.station.model.Channel;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.text.ParseException;
import java.util.Date;
import java.util.TimeZone;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
// Import the error handling classes
// Import the classes specific to the STATION example
// Import the classes specific to the WAVEFORM example
// Import the classes specific to the EVENT example
// Import other Java classes>
//
public class TraceQuery {
  public static void main(String[] args) {
    // get access to the services
    // specify criteria
    // fetch data
    // process data
    TraceData traceFetcher = new TraceData();
    traceFetcher.setAppName("ALarson, APL-UW");
    
    // define the dates of interest
    DateFormat dfm = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss");
    
    //dfm.setTimeZone(TimeZone.getTimeZone("UTC"));
    //
    try {
        
        Date startDate = dfm.parse("2023-03-17T22:00:00");
        Date endDate = dfm.parse("2023-03-19T02:00:00");
        
        
        
        
        System.out.printf("Querying for date from\n\t%s to %s\n\n", dfm.format(startDate), dfm.format(endDate));
        Boolean includePZ = false;
        Trace traces []=null;  
        String NETWORK = "IM";
        String STATION = "H11N2";
        
        
        String LOCATION = "*";
        String CHANNEL = "EDH";
        // Traces are unique in that you could provide a station criteria object (as in the station example) or
        // you can specify simple criteria directly in the call.</span>


        TraceData.setBASE_URL("https://service.iris.edu");
        traces = traceFetcher.fetchTraces(NETWORK,STATION,LOCATION,CHANNEL,
                 startDate, endDate, 'M', includePZ);
        

//loop through all the channels to display the details
        for (Trace trace : traces) {
            
            
            String nw = trace.getNetwork();
            String station = trace.getStation();
            double FS = trace.getSampleRate();
            double sensitivity = trace.getSensitivity();
            
            // Note this date string seems to be stuck in PST
            String filename=trace.getStartTime().toString() + ".txt";
            
            System.out.println("Working Directory = " + System.getProperty("user.dir"));
            Path fpath = Paths.get("..", "data", "IRIS", nw, station, filename);
            System.out.printf("writing to file path: %s\n", fpath.toString());
            System.out.printf("Found %2s-%5s (%2s)  from %15s to %15s\n",
              nw,station,trace.getChannel(),
              trace.getStartTime().toString(), trace.getEndTime().toString());
            System.out.printf("  This trace has %d samples, %7.2f samples per second, instrument: %s, sensitivity: %f [counts / Pa], quality: %c \n",
                   trace.getSampleCount(), FS, trace.getInstrument(), sensitivity, trace.getQuality());
            System.out.printf("  Channel location at: %8.4f lat, %8.4f lon, elev %.0f m\n\n",
              trace.getLatitude(), trace.getLongitude(), trace.getElevation());
            
           
            try {
                System.out.printf("Trace: %s\n", trace.toString());
                FileWriter writer = new FileWriter(fpath.toString());
                writer.write("#Network\tStation\tStarttime\t\tFS\t\tUnits\tSensitivity\n");
                writer.write(String.format("#%s\t\t%s\t%s\t%f\t%s\t%f\n",
                        nw, station, trace.getStartTime().toString(), FS, "uPa", sensitivity));
                for (double val : trace.getAsDouble()) {
                    writer.write(String.format("%f\n", val));
                }

                writer.close();
                System.out.println("Successfully wrote to the file.");
            } catch (IOException e) {
                System.out.println("An error occurred.");
                e.printStackTrace();
            }
        }
        
    } catch (Exception pe) {
        System.out.printf("Exception in main: %s\n", pe.toString());
    }
  }
}
