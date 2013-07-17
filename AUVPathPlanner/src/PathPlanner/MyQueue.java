/*
 * MyQueue class
 * 
 * Implements the MyQueue interface which has the methods
 *     isEmpty()
 *     peek()
 *     dequeue()
 *     enqueue()
 * 
 */
package PathPlanner;

/**
 *
 * @author atthewco
 */
public class MyQueue {
    //QCell is inner class to hold queue objects
    class QCell {
    private Object data;
    private QCell next;

    private QCell(Object data)
    {
      this.data = data;
      this.next = null;
    }
    }

  /* data members of the Queue
  */
  private QCell front;  // the front of the queue for dequeuing
  private QCell back;  // the front of the queue for enqueuing

  /* constructor
   * creates and empty stack
   */
  public MyQueue()
  {
    this.front = this.back = null;
  }

  /* method: isEmpty
   * inputs: none
   * outputs: a boolean - whether or not this is empty
   */
  public boolean isEmpty()
  {
    return (front == null);
  }

  /* method: enqueue
   * inputs: an object to store at the back of the queue
   * outputs: none
   */
  public void enqueue(Object data)
  {
    QCell newBack = new QCell(data);
    if (isEmpty())
    {
      this.front = this.back = newBack;
    }
    else
    {
      this.back.next = newBack;
      this.back = newBack;
    }
  }

  /* method: dequeue
   * inputs: none
   * outputs: the data that was at the front (null if empty)
   *          pop also removes the front QCell
   */
  public Object dequeue()
  {
    if (isEmpty())
    {
      System.out.println("You can't dequeue from an empty Queue!");
      return null;
    }
    Object data = front.data;
    front = front.next;
    if (front == null) this.back = null; // clear the back, if needed
    return data;
  }

  /* method: peek
   * inputs: none
   * outputs: the data that was at the front (null if empty)
   *          peek does not alter the calling Queue
   */
  public Object peek()
  {
    if (isEmpty())
    {
      System.out.println("You can't peek at an empty Queue!");
      return null;
    }
    Object data = front.data;
    return data;
  }

  /* method: toString
   * inputs: none
   * outputs: the usual
   */
  public String toString()
  {
    String result = "<FRONT> ";
    QCell current = this.front;
    while (current != null)
    {
      result += "" + current.data + " ";
      current = current.next;
    }
    result += "<BACK>";
    return result;
  }
}

