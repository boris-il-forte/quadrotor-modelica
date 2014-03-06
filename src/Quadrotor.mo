model Quadrotor
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Chassis chassis1 annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-26.25,-26.25},{26.25,26.25}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantvoltage1(V = 12) annotation(Placement(visible = true, transformation(origin = {-80,-60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-80,-100}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Arm arm2 annotation(Placement(visible = true, transformation(origin = {-60,0}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = 0)));
  Arm arm3 annotation(Placement(visible = true, transformation(origin = {0,-60}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = -90)));
  Arm arm4 annotation(Placement(visible = true, transformation(origin = {0,60}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = -90)));
  Arm arm1 annotation(Placement(visible = true, transformation(origin = {60,0}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = 0)));
equation
  connect(constantvoltage1.p,arm1.pin_p) annotation(Line(points = {{-80,-50},{-80,-47.3684},{30.5263,-47.3684},{30.5263,-9.47368},{36.3158,-9.47368},{36.3158,-9.47368}}));
  connect(constantvoltage1.p,arm4.pin_p) annotation(Line(points = {{-80,-50},{-80,49.4737},{-8.947369999999999,49.4737},{-8.947369999999999,37.3684},{-8.947369999999999,37.3684}}));
  connect(constantvoltage1.p,arm3.pin_p) annotation(Line(points = {{-80,-50},{-80,-48.4211},{-9.47368,-48.4211},{-9.47368,-38.9474},{-9.47368,-38.9474}}));
  connect(constantvoltage1.p,arm2.pin_p) annotation(Line(points = {{-80,-50},{-80,-8.947369999999999},{-38.4211,-8.947369999999999},{-38.4211,-8.947369999999999}}));
  connect(constantvoltage1.n,arm4.pin_n) annotation(Line(points = {{-80,-70},{-80,-78.4211},{-18.9474,-78.4211},{-18.9474,39.4737},{-18.9474,39.4737}}));
  connect(constantvoltage1.n,arm1.pin_n) annotation(Line(points = {{-80,-70},{-80,-85.2632},{37.3684,-85.2632},{37.3684,-16.8421},{37.3684,-16.8421}}));
  connect(constantvoltage1.n,arm2.pin_n) annotation(Line(points = {{-80,-70},{-80,-74.2105},{-37.8947,-74.2105},{-37.8947,-17.8947},{-37.8947,-17.8947},{-37.8947,-17.8947}}));
  connect(arm3.pin_n,constantvoltage1.n) annotation(Line(points = {{-18,-37.5},{-18,-70.52630000000001},{-79.47369999999999,-70.52630000000001},{-79.47369999999999,-70.52630000000001}}));
  connect(arm3.frame_a,chassis1.frame_S) annotation(Line(points = {{-1.37773e-15,-37.5},{-1.37773e-15,-25.7895},{-1.05263,-25.7895},{-1.05263,-25.7895}}));
  connect(chassis1.frame_W,arm1.frame_a) annotation(Line(points = {{26.25,0},{37.3684,0},{37.3684,1.05263},{37.3684,1.05263}}));
  connect(chassis1.frame_N,arm4.frame_a) annotation(Line(points = {{0,26.25},{0,26.25},{0,37.8947},{0,37.8947}}));
  connect(arm2.frame_a,chassis1.frame_E) annotation(Line(points = {{-37.5,0},{-26.3158,0},{-26.3158,0.526316},{-26.3158,0.526316}}));
  connect(ground1.p,constantvoltage1.n) annotation(Line(points = {{-80,-90},{-80,-90},{-80,-69.47369999999999},{-80,-69.47369999999999}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Quadrotor;