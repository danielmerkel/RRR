using UnityEngine;
using System.Collections.Generic;

public class CarController : MonoBehaviour
{

	// Este script do carro é projetado para ser usado em um GameObject que tem rodas presas. 
	// As rodas devem ser objetos filhos, e cada uma tem um script roda em anexo, e um componente WheelCollider.

	// Mesmo que wheelcolliders têm suas próprias configurações para a perda de aderência, este script de carro
	// (e seus scripts roda de acompanhamento) modificam as configurações nas wheelcolliders em tempo de execução,
	// para dar uma experiência mais exagerada e divertido, permitindo comportamento de uma
	// forma que não é prontamente realizável usando valores constantes em wheelcolliders sozinho.

	// As prioridades de código e diversão sobre realismo, e apesar de um sistema de marchas está incluído, 
	// não é usada para acionar o motor. Em vez disso, as rotações e equipamentos atuais são calculados 
	// retrospectivamente com base na velocidade atual do carro. Estes valores de marchas e rev pode ser
	//lido e utilizado por uma interface gráfica ou componente de som.


    [SerializeField] private float maxSteerAngle = 28;                              // o angulo maximo que o carro pode dobrar
    [SerializeField] private float steeringResponseSpeed = 200;                     // o quanto rapido o volante pode responder
    [SerializeField] [Range(0, 1)] private float maxSpeedSteerAngle = 0.23f;        // reduz o angulo de curva na velocidade maxima
    [SerializeField] [Range(0, .5f)] private float maxSpeedSteerResponse = 0.5f;    // reduz a resposta do volante na veolcidade maxima
    [SerializeField] private float maxSpeed = 60;                                   // velocidade maxima (metros por segundo)
    [SerializeField] private float maxTorque = 35;                                  // traçao maxima do motor
    [SerializeField] private float minTorque = 10;                                  // traçao minima do motor
    [SerializeField] private float brakePower = 40;                                 // força dos freios
    [SerializeField] private float adjustCentreOfMass = 0.25f;                      // alinhamento vertical do centro de massa
    [SerializeField] private Advanced advanced;                                     // container for the advanced setting which will expose as a foldout in the inspector
	[SerializeField] bool preserveDirectionWhileInAir = true;                       // ajuda o carro a pousar na direçao certa se ele pular)


    [System.Serializable]
    public class Advanced                                                           // configuraçoes avançadas de controle do carro
    {
        [Range(0, 1)] public float burnoutSlipEffect = 0.4f;                        // o quanto as rodas do carro vao deslizar qdo queimar pneu
        [Range(0, 1)] public float burnoutTendency = 0.2f;                          // probabilidade do carro queimar pneu 
        [Range(0, 1)] public float spinoutSlipEffect = 0.5f;                        // o quanto facilmente o carro vai girar quando estiver dobrando
        [Range(0, 1)] public float sideSlideEffect = 0.5f;                          // o qunato facilmente o carro perde a aderencia lateral 

        public float downForce = 30;                                                // a quantidade de força que eh aplicada (speed is factored in)
        public int numGears = 5;                                                    // o numero de marchas
        [Range(0, 1)] public float gearDistributionBias = 0.2f;                     // Controls whether the gears are bunched together towards the lower or higher end of the car's range of speed.
		public float steeringCorrection = 2f;                                       // O quao rapido o volante retorna ao centro sem input de controle
		public float oppositeLockSteeringCorrection = 4f;                           // O quanto rápido é a resposta da direção qdo o carro tem input na direção oposta do atual angulo da roda
        public float reversingSpeedFactor = 0.3f;                                   // faor de velocida maxima de re, proporcional a velocdiade maxima de frente.
        public float skidGearLockFactor = 0.1f;                                     // O carro nao vai trocar automaticamente a marcha se o faot de derrapagem for maior que esse valor.
        public float accelChangeSmoothing = 2f;                                     // Usado para suavizar as mudanças de  imputs de aceleraçao.
		public float gearFactorSmoothing = 5f;                                      // CControla a velocidade com a qual acelera ou diminui a nova marcha, depois de uma mudança de marça.
        [Range(0,1)]public float revRangeBoundary = 0.8f;                           // O valor da gama de rotaçao usado em cada marcha.
    }


	private float[] gearDistribution;                                               // Armazena o ponto de mudança calculado para cada marcha (0-1 como uma quantidade normalizada em relação à velocidade máxima do carro)
	private Wheel[] wheels;                                                         // Armazena uma referência para cada roda ligado a este carro..
	private float accelBrake;                                                       // A aceleração de frenagem de entrada ( intervalo 1, -1)
	private float smallSpeed;                                                       // Uma pequena proporção de velocidade máxima, usado para decidir quando começar a aceleração / travagem quando a transição entre o moviemnto frente e tras.
	private float maxReversingSpeed;                                                // velocida maxima de re
	private bool immobilized;                                                       // Se o carro é aceitar input.


	// publicly read-only props, useful for GUI, Sound effects, etc.
	public int GearNum { get; private set; }                                        // a marcha atual
	public float CurrentSpeed { get; private set; }                                 // a velocidade aatual do carro
	public float CurrentSteerAngle{ get; private set; }                             // o angulo de direçao para as rodas direcionaveis
	public float AccelInput { get; private set; }                                   // o imput de aceleracao atual
	public float BrakeInput { get; private set; }                                   // o imput de freio atual
	public float GearFactor  { get; private set; }                                  // valor entre 0-1 indicando onde as rotações atuais são da actual intervalo de rotações dsta marcha
	public float AvgPowerWheelRpmFactor { get; private set; }                       // a RPM media de todas as rodas marcadas como  'tracionadas'
	public float AvgSkid { get; private set; }                                      // o fator medio de derrapagem para todas as rodas
	public float RevsFactor { get; private set; }                                   // valor entre 0-1 indicando onde as rotações atuais caiem entre 0 e max rotações
	public float SpeedFactor { get;  private set; }                                 // valor entre 0-1 da velocidade do carro atual em relação à velocidade máxima

	public int CurrentLaps {get; private set;}    									// A lap atual que o carro se encontra.
	public bool colliderPassed {get; private set;}									// Se o collider car ja passou pelo collider de checkpoint.
	public string LastCheckpoint {get;private set;}
	public int managerCheckpoint {get;set;}
	public List<bool> CheckpointPass {get;set;}

	public int NumGears {															// the numero de marchas configurado no carro
		get { return advanced.numGears; }
	}						


	// os seguintes valores são fornecidos como propriedades somente leitura, 
	// e são necessários pelo script Wheel para calcular aderência, neutralização, derrapagem, etc

    public float MaxSpeed
    {
        get { return maxSpeed; }
    }


    public float MaxTorque
    {
        get { return maxTorque; }
    }


    public float BurnoutSlipEffect
    {
        get { return advanced.burnoutSlipEffect; }
    }


    public float BurnoutTendency
    {
        get { return advanced.burnoutTendency; }
    }


    public float SpinoutSlipEffect
    {
        get { return advanced.spinoutSlipEffect; }
    }


    public float SideSlideEffect
    {
        get { return advanced.sideSlideEffect; }
    }


    public float MaxSteerAngle
    {
        get { return maxSteerAngle; }
    }
    


	bool anyOnGround;
	float curvedSpeedFactor;
	bool reversing;
	float targetAccelInput; //entrada de aceleração alvo é a nossa entrada aceleração desejada. 





	void Awake ()
    {

		// Obtém uma referência para todas as rodas atachadas ao carro.
		wheels = GetComponentsInChildren<Wheel>();

		SetUpGears();


		// Desativar e reativar o GameObject - esta é uma solução alternativa 
		// para um bug onde mudanças para wheelcolliders em tempo de execução não são atualziadas
		// pelo rigidbody a menos que este passo é realizado.

		gameObject.SetActive(false);
		gameObject.SetActive(true);

		//algumas velocidades uteis calculadas para uso posterior
	    smallSpeed = maxSpeed*0.05f;
        maxReversingSpeed = maxSpeed * advanced.reversingSpeedFactor;

		this.LastCheckpoint = "";
		this.CheckpointPass = new List<bool>();
		this.CheckpointPass.Add(false);
		this.CheckpointPass.Add(false);
		this.CheckpointPass.Add(false);
		this.CheckpointPass.Add(false);
	}


	void OnEnable()
	{
		//define o centro de massa
		rigidbody.centerOfMass = Vector3.up * adjustCentreOfMass;
	}


	public void Move (float steerInput, float accelBrakeInput)
    {

		// perde o controle do motor se estiver imobilizado
		if (immobilized) accelBrakeInput = 0;

		ConvertInputToAccelerationAndBraking (accelBrakeInput);
		CalculateSpeedValues ();
		HandleGearChanging ();
		CalculateGearFactor ();
		ProcessWheels (steerInput);
        ApplyDownforce ();
		CalculateRevs();
		PreserveDirectionInAir();

	}

	void ConvertInputToAccelerationAndBraking (float accelBrakeInput)
	{

		// Move.Z é o imput de usuario frente/tras. Nós precisamos convertê-lo em aceleração e frenagem. 
		// Isto varia conforme o se o carro está se movendo para a frente ou para trás. 
		// A troca baseia-se um pouco longe do valor zero (por "smallspeed"), de modo que, por exemplo, quando 
		// o movimento do carro reverter para frente, o carro não precisa  parar antes de começar a acelerar.


		reversing = false;
		if (accelBrakeInput > 0) {
			if (CurrentSpeed > -smallSpeed) {
				// pressionando para frente enquanto anda pra frente : acelerar!
				targetAccelInput = accelBrakeInput;
				BrakeInput = 0;
			}
			else {
				// presisonando para frente enqunato anda pra tras: freiar!!
				BrakeInput = accelBrakeInput;
				targetAccelInput = 0;
			}
		}
		else {
			if (CurrentSpeed > smallSpeed) {
				// pressionando para tras enquanto anda para frente : freiar!
				BrakeInput = -accelBrakeInput;
				targetAccelInput = 0;
			}
			else {
				// pressionando para tras enquanto move para tras : acelerar em re!
				BrakeInput = 0;
				targetAccelInput = accelBrakeInput;
				reversing = true;
			}
		}
		//mover suavemente a aceleração atual para o valor de aceleração alvo.
		AccelInput = Mathf.MoveTowards (AccelInput, targetAccelInput, Time.deltaTime * advanced.accelChangeSmoothing);
	}

	void CalculateSpeedValues ()
	{
		// Velocidade atual é medida na direção a frente do carro (deslizamento lateral não conta!)
		CurrentSpeed = transform.InverseTransformDirection (rigidbody.velocity).z;
		// Speedfactor é uma representação normalizada de velocidade em relação à velocidade máxima:
		SpeedFactor = Mathf.InverseLerp (0, reversing ? maxReversingSpeed : maxSpeed, Mathf.Abs (CurrentSpeed));
		curvedSpeedFactor = reversing ? 0 : CurveFactor (SpeedFactor);
	}

	void HandleGearChanging ()
	{

		//troca de marcha, quando for o caso (se a velocidade subiu acima ou abaixo da faixa da marcha atual armazenada na matriz gearDistribution)
		if (!reversing) {
			if (SpeedFactor < gearDistribution [GearNum] && GearNum > 0)
				GearNum--;
			if (SpeedFactor > gearDistribution [GearNum + 1] && AvgSkid < advanced.skidGearLockFactor && GearNum < advanced.numGears - 1)
				GearNum++;
		}
	}

	void CalculateGearFactor ()
	{
		//gear factor é uma representação normalizada da velocidade atual dentro do alcance da marcha atual de velocidades. 

		//Nos suavizamos o alvo do gear factor, de modo que as rotações não trocam instantaneamente para cima ou para baixo quando troca a marcha
		var targetGearFactor = Mathf.InverseLerp (gearDistribution [GearNum], gearDistribution [GearNum + 1], Mathf.Abs (AvgPowerWheelRpmFactor));
		GearFactor = Mathf.Lerp (GearFactor, targetGearFactor, Time.deltaTime * advanced.gearFactorSmoothing);
	}

	void ProcessWheels (float steerInput)
	{
		// processa cada roda

		AvgPowerWheelRpmFactor = 0;
		AvgSkid = 0;
		var numPowerWheels = 0;
		anyOnGround = false;
		foreach (var wheel in wheels) {
			var wheelCollider = wheel.wheelCollider;
			if (wheel.steerable) {
				//aplicar direção a esta roda. A mudança de direcção baseia-se no range de direcção, velocidade actual,
				//e se a roda está apontando na direcção que está a sendo aplicado
				var currentSteerSpeed = Mathf.Lerp (steeringResponseSpeed, steeringResponseSpeed * maxSpeedSteerResponse, curvedSpeedFactor);
				var currentMaxAngle = Mathf.Lerp (maxSteerAngle, maxSteerAngle * maxSpeedSteerAngle, curvedSpeedFactor);
				//auto-correção da direçao para o centro, se nenhuma entrada de direçao foi feita:
				if (steerInput == 0) {
					currentSteerSpeed *= advanced.steeringCorrection;
				}

				//aumentar a velocidade de direção se a entrada de direção está na direção oposta à direção da roda atual (para uma resposta mais rápida)

				if (Mathf.Sign (steerInput) != Mathf.Sign (CurrentSteerAngle)) {
					currentSteerSpeed *= advanced.oppositeLockSteeringCorrection;
				}
				//Modificar o ângulo de direcção efectiva da roda por estes valores calculados:
				CurrentSteerAngle = Mathf.MoveTowards (CurrentSteerAngle, steerInput * currentMaxAngle, Time.deltaTime * currentSteerSpeed);
				wheelCollider.steerAngle = CurrentSteerAngle;
			}

			//acumular a quantidade de derrapagem dessa roda, para mais tarde
			AvgSkid += wheel.SkidFactor;
			if (wheel.powered) {
				//aplica força para as rodas marcadas como tracionadas
				//torque disponível cai à medida que se aproxima a velocidade máxima
				var currentMaxTorque = Mathf.Lerp (maxTorque, (SpeedFactor < 1) ? minTorque : 0, reversing ? SpeedFactor : curvedSpeedFactor);
				wheelCollider.motorTorque = AccelInput * currentMaxTorque;
				//Acumular RPM  desta roda, para uma média de mais tarde
				AvgPowerWheelRpmFactor += wheel.Rpm / wheel.MaxRpm;
				numPowerWheels++;
			}

			// aplicar força de freio para a roda atual
			wheelCollider.brakeTorque = BrakeInput * brakePower;
			// se qualqeur roda estiver tocando o solo, o carro e considerado no chao
			if (wheel.OnGround) {
				anyOnGround = true;
			}
		}
		//media dos valroes acumulados da roda
		AvgPowerWheelRpmFactor /= numPowerWheels;
		AvgSkid /= wheels.Length;
	}

	void ApplyDownforce ()
	{
		// aplicar força pra baixo
		if (anyOnGround) {
			rigidbody.AddForce (-transform.up * curvedSpeedFactor * advanced.downForce);
		}
	}

	void CalculateRevs ()
	{
		// calcular rotacoes do movotr (para mostrar / som)
		// (isto é feito em retrospecto - rotações não são usados ​​nos cálculos de força / potência)
		var gearNumFactor = GearNum / (float)NumGears;
		var revsRangeMin = ULerp (0f, advanced.revRangeBoundary, CurveFactor (gearNumFactor));
		var revsRangeMax = ULerp (advanced.revRangeBoundary, 1f, gearNumFactor);
		RevsFactor = ULerp (revsRangeMin, revsRangeMax, GearFactor);
	}

	void PreserveDirectionInAir()
	{
		// permite que o carro mantenha-se apontando para a direçao que eles esta
		if (!anyOnGround && preserveDirectionWhileInAir && rigidbody.velocity.magnitude > smallSpeed) {
			rigidbody.MoveRotation (Quaternion.Slerp (rigidbody.rotation, Quaternion.LookRotation (rigidbody.velocity), Time.deltaTime));
			rigidbody.angularVelocity = Vector3.Lerp (rigidbody.angularVelocity, Vector3.zero, Time.deltaTime);
		}
	}

	//função simples para adicionar uma curva de desvio em direcção um para um valor no intervalo de 0-1

    float CurveFactor (float factor)
    {
        return 1 - (1 - factor)*(1 - factor);
    }
	

	// unclamped versao of Lerp, para permtir values excedam a partir do intervalo
	float ULerp (float from, float to, float value)
    {
		return (1.0f - value)*from + value*to;
	}


	void SetUpGears()
	{

		// a distribuição de marchas é uma intervalo de valores normalizados que marcam para onde as mudanças de velocidade devem ocorrer
		// ao longo do intervalo normalizado de velocidades para o carro. 
		// por exemplo, se o desvio é centrado, 5 engrenagens seriam distribuídas uniformemente como 0-0,2, 0,2-0,4, 0,4-0,6, 0,6-0,8, 0,8-1 
		// com um desvio baixo, as engrenagens são aglutinados para a extremidade inferior do intervalo de velocidades, e vice-versa em alta polarização.


        gearDistribution = new float[advanced.numGears + 1];
        for (int g = 0; g <= advanced.numGears; ++g)
		{
            float gearPos = g / (float)advanced.numGears;

			float lowBias = gearPos*gearPos*gearPos;
            float highBias = 1 - (1 - gearPos) * (1 - gearPos) * (1 - gearPos);

			if (advanced.gearDistributionBias < 0.5f)
			{
				gearPos = Mathf.Lerp(gearPos, lowBias, 1 - (advanced.gearDistributionBias * 2));
			} else {
                gearPos = Mathf.Lerp(gearPos, highBias, (advanced.gearDistributionBias - 0.5f) * 2);
			}

			gearDistribution[g] = gearPos;
		}
	}

	
	void OnDrawGizmosSelected()
	{
		// visualize o ajuste do centro de massas no editor
		Gizmos.color = Color.cyan;
		Gizmos.DrawWireSphere(rigidbody.position + Vector3.up * adjustCentreOfMass, 0.2f);
	}

	public void OnTriggerEnter(Collider other)
	{
		switch (other.name) {
			case("CheckpointA"):
			if(!this.LastCheckpoint.Equals(other.name))
			{
				if(this.LastCheckpoint.Equals("") || this.CurrentLaps == 0)
				{
					this.CurrentLaps++;
					this.CheckpointPass[0] = true;
				}
				else 
				{
					var updateLap = true;
					foreach (var passou in this.CheckpointPass) {
						if(!passou){
							updateLap= false;
						}
					}
				

					if(updateLap){
						this.CurrentLaps++;
						for (int i = 0; i < this.CheckpointPass.Count; i++) {
							this.CheckpointPass[i] = false;
						}
						this.CheckpointPass[0] = true;
					}

				}
				this.LastCheckpoint = other.name;
			}
			break;
			case("CheckpointB"):
			if(!this.LastCheckpoint.Equals(other.name))
			{
//				if(!this.colliderPassed)
//				{
//					this.managerCheckpoint = 1;
//					this.colliderPassed = true;				
//				}
				this.CheckpointPass[1] = true;
				this.LastCheckpoint = other.name;
			}
			break;
			case("CheckpointC"):
			if(!this.LastCheckpoint.Equals(other.name))
			{
//				if(!this.colliderPassed)
//				{
//					this.managerCheckpoint = 2;
//					this.colliderPassed = true;				
//				}
				this.CheckpointPass[2] = true;
				this.LastCheckpoint = other.name;
			}
			break;
			case("CheckpointD"):
			if(!this.LastCheckpoint.Equals(other.name))
			{
//				if(!this.colliderPassed)
//				{
//					this.managerCheckpoint = 3;
//					this.CurrentLaps++;
//					this.colliderPassed = true;				
//				}
				this.CheckpointPass[3] = true;
				this.LastCheckpoint = other.name;
			}
			break;
			default:
			break;
		}




		var gameObjectCollider = GameObject.Find("Collider2");
		var teste = gameObject.name;


	}

	// Immobilizacao pode ser chamaado por outros objetos, seo cara precisa ser incontrolavel (ex: explosao)
	public void Immobilize ()
	{
		immobilized = true;
	}
	
	// Reset e chamado pelo script ObjectResetter script, se houver.
	public void Reset()
	{
		immobilized = false;
	}
}
