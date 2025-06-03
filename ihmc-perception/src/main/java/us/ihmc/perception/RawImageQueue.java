package us.ihmc.perception;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

public class RawImageQueue
{
   private final BlockingQueue<RawImage> queue;

   public RawImageQueue()
   {
      // You can specify a bounded capacity if needed.
      this.queue = new LinkedBlockingQueue<>();
   }

   /**
    * Adds an image to the queue. This will not block unless a capacity limit is set and reached.
    */
   public void enqueue(RawImage image)
   {
      if (image != null)
         queue.offer(image); // offer is non-blocking
   }

   /**
    * Retrieves and removes the next image from the queue, waiting if necessary
    * until an image becomes available.
    */
   public RawImage dequeue() throws InterruptedException
   {
      return queue.take(); // this blocks if queue is empty
   }

   /**
    * Returns the number of images currently in the queue.
    */
   public int size()
   {
      return queue.size();
   }

   /**
    * Retrieves and removes the next image from the queue, or returns null if no image is available within the timeout.
    */
   public RawImage poll(long timeout, TimeUnit unit) throws InterruptedException
   {
      return queue.poll(timeout, unit);
   }

   /**
    * Clears all images currently in the queue.
    */
   public void clear()
   {
      queue.clear();
   }
}