package us.ihmc.perception;

import us.ihmc.sensors.ImageSensor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PerceptionImageGrabber
{
   private final RawImageQueue rawImageQueue;
   private final ExecutorService sensorExecutor;

   public PerceptionImageGrabber(RawImageQueue rawImageQueue)
   {
      this.rawImageQueue = rawImageQueue;
      this.sensorExecutor = Executors.newCachedThreadPool(); // One thread per sensor
   }

   public void addImageSensor(ImageSensor imageSensor, int imageKey)
   {
      sensorExecutor.submit(() ->
                            {
                               while (!Thread.currentThread().isInterrupted())
                               {
                                  try
                                  {
                                     // Wait for new image to be available
                                     imageSensor.waitForGrab();

                                     // Get the image
                                     RawImage rawImage = imageSensor.getImage(imageKey);
                                     if (rawImage != null)
                                     {
                                        rawImageQueue.enqueue(rawImage);
                                     }
                                  }
                                  catch (InterruptedException e)
                                  {
                                     // Exit the thread cleanly on interruption
                                     Thread.currentThread().interrupt();
                                  }
                                  catch (Exception e)
                                  {
                                     e.printStackTrace(); // Log or handle image sensor failure
                                  }
                               }
                            });
   }

   public void shutdown()
   {
      sensorExecutor.shutdownNow();
   }
}
