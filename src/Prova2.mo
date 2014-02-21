model Prova2
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantvoltage1 annotation(Placement(visible = true, transformation(origin = {-60,60}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-12.5,-12.5},{12.5,12.5}}, rotation = 0)));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(VaNominal = 12, IaNominal = 1, wNominal = 314) annotation(Placement(visible = true, transformation(origin = {-55,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.LossyGear lossygear1(ratio = 1) annotation(Placement(visible = true, transformation(origin = {5,15}, extent = {{-15,-15},{15,15}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angle annotation(Placement(visible = true, transformation(origin = {60,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor angularSpeed annotation(Placement(visible = true, transformation(origin = {60,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Rotor rotor1 annotation(Placement(visible = true, transformation(origin = {20,-60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
equation
  connect(rotor1.frame_b,world.frame_b) annotation(Line(points = {{10,-60},{10,-60.2291},{-49.7545,-60.2291},{-49.7545,-60.2291}}));
  connect(rotor1.flange_a,lossygear1.flange_b) annotation(Line(points = {{20,-50},{20,15.0573},{20.2946,15.0573},{20.2946,15.0573}}));
  connect(lossygear1.flange_b,angle.flange) annotation(Line(points = {{20,15},{20.4268,15},{20.4268,40.2439},{50,40.2439},{50,40.2439}}));
  connect(angularSpeed.flange,lossygear1.flange_b) annotation(Line(points = {{50,0},{20.4268,0},{20.4268,15.2439},{20.4268,15.2439}}));
  connect(dcpm.support,world.frame_b) annotation(Line(points = {{-40,0},{-23.7805,0},{-23.7805,-60.3659},{-50,-60.3659},{-50,-60.3659}}));
  connect(dcpm.support,lossygear1.support) annotation(Line(points = {{-40,0},{3.96341,0},{3.96341,-0.304878},{3.96341,-0.304878}}));
  connect(dcpm.flange,lossygear1.flange_a) annotation(Line(points = {{-40,15},{-10.6707,15},{-10.6707,14.6341},{-10.6707,14.6341}}));
  connect(dcpm.pin_ap,constantvoltage1.p) annotation(Line(points = {{-46,30},{-39.3293,30},{-39.3293,60.3659},{-50,60.3659},{-50,60}}));
  connect(constantvoltage1.n,dcpm.pin_an) annotation(Line(points = {{-70,60},{-79.5732,60},{-79.5732,30.1829},{-66,30},{-64,30}}));
  connect(ground1.p,constantvoltage1.n) annotation(Line(points = {{-60,92.5},{-70.4268,92.5},{-70.4268,60.6707},{-70.4268,60.6707}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Prova2;