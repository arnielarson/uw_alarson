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
import edu.iris.dmc.criteria.OutputLevel;
import edu.iris.dmc.criteria.StationCriteria;
import edu.iris.dmc.extensions.entities.Trace;
// error handling
//<i>... more here...
//
// ServiceUtil is where we obtain all the "hooks" to the web services.
import edu.iris.dmc.service.ServiceUtil;
import edu.iris.dmc.service.StationService;
import edu.iris.dmc.extensions.fetch.TraceData;
import edu.iris.dmc.fdsn.station.model.Channel;
import edu.iris.dmc.fdsn.station.model.Network;
import edu.iris.dmc.fdsn.station.model.Response;
import edu.iris.dmc.fdsn.station.model.Station;
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
public class ResponseQuery {
  public static void main(String[] args) {
    // get access to the services
    // specify criteria
    // fetch data
    // process data
    ServiceUtil serviceUtil = ServiceUtil.getInstance();
    serviceUtil.setAppName("ALarson, APL-UW");
    StationService stationService = serviceUtil.getStationService();
    String NETWORK = "IM";
    String STATION = "*";
    String CHANNEL = "EDH";
    
    
    try {
       // define the dates of interest</span>
        DateFormat dfm = new SimpleDateFormat("yyyy-MM-dd");
        dfm.setTimeZone(TimeZone.getTimeZone("PST"));
        //
        Date startDate = dfm.parse("2004-01-01");
        Date endDate = dfm.parse("2023-01-01");
        //
        // specify the search criteria</span>
        StationCriteria stationCriteria = new StationCriteria();
        stationCriteria = stationCriteria.addNetwork(NETWORK).addStation(STATION).addChannel(CHANNEL).
        setStartAfter(startDate).setEndBefore(endDate);
        
        List<Network> ls = stationService.fetch(stationCriteria, OutputLevel.RESPONSE);
    
        // loop through all the channels to display the details
        for (Network n : ls) {
                System.out.printf("Network : %s with %d stations\n", n.getCode(),
                                n.getSelectedNumberStations());
                // each network may have multiple stations
                for (Station s : n.getStations()) {
                        //System.out.printf("\tStation : %s with %d channels\n",
                        //                s.getCode(), s.getChannels().size());
                        System.out.println(s.toString());
                        try{ 
                            List<Channel> channels = s.getChannels();
                            if (null != channels) {
                                for (Channel c : channels) {
                                    // Finally! Print the detail
                                    //System.out.printf("\t\tChannel: %s on: %s   off:%s\n",
                                    //                c.getCode(), c.getStartDate(), c.getEndDate());
                                    System.out.println(c.toString());
                                    Response r = c.getResponse();
                                    System.out.printf("\tResponse: %s\n",r.toString());
                                }
                            }
                        } catch (Exception e) {
                            System.out.println(e.toString());   
                        }
                }
        }
    } catch (Exception pe) {
        System.out.println("Exception in Main");
    }
  }
}
