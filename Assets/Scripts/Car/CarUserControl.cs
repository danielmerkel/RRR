using UnityEngine;

[RequireComponent(typeof(CarController))]
public class CarUserControl : MonoBehaviour
{
    private CarController car;  // the car controller we want to use
    

    void Awake ()
    {
        // get the car controller
        car = GetComponent<CarController>();
    }


    void FixedUpdate()
    {
        // pass the input to the car!

		float h = Input.GetAxis("Horizontal");
		float v = Input.GetAxis("Vertical");

        car.Move(h,v);
    }


}
