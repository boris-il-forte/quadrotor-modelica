model Quadrotor
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Arm arm1 annotation(Placement(visible = true, transformation(origin = {80,20}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = 0)));
  Arm arm2 annotation(Placement(visible = true, transformation(origin = {-60,20}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = 0)));
  Arm arm3 annotation(Placement(visible = true, transformation(origin = {20,-60}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = -90)));
  Arm arm4 annotation(Placement(visible = true, transformation(origin = {20,80}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = -90)));
  Chassis chassis1 annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-26.25,-26.25},{26.25,26.25}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantvoltage1 annotation(Placement(visible = true, transformation(origin = {-80,-60}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-80,-100}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(constantvoltage1.p,arm3.pin_p) annotation(Line(points = {{-80,-50},{-80,-28.4211},{33.1579,-28.4211},{33.1579,-36.8421},{33.1579,-36.8421}}));
  connect(arm1.pin_p,constantvoltage1.p) annotation(Line(points = {{57.5,33.5},{-20.5263,33.5},{-20.5263,57.8947},{-90,57.8947},{-90,-49.4737},{-80.52630000000001,-49.4737},{-80.52630000000001,-49.4737}}));
  connect(constantvoltage1.p,arm4.pin_p) annotation(Line(points = {{-80,-50},{-80,51.0526},{33.1579,51.0526},{33.1579,57.3684},{33.1579,57.3684}}));
  connect(constantvoltage1.p,arm2.pin_p) annotation(Line(points = {{-80,-50},{-80,42.6316},{-36.8421,42.6316},{-36.8421,34.7368},{-36.8421,34.7368}}));
  connect(constantvoltage1.n,arm4.pin_n) annotation(Line(points = {{-80,-70},{-80,-80},{38.4211,-80},{38.4211,38.9474},{24.7368,38.9474},{24.7368,57.8947},{24.7368,57.8947}}));
  connect(constantvoltage1.n,arm1.pin_n) annotation(Line(points = {{-80,-70},{-80,-74.2105},{58.9474,-74.2105},{58.9474,25.7895},{58.9474,25.7895}}));
  connect(constantvoltage1.n,arm3.pin_n) annotation(Line(points = {{-80,-70},{-80,-69.47369999999999},{-7.89474,-69.47369999999999},{-7.89474,-36.8421},{25.2632,-36.8421},{25.2632,-36.8421}}));
  connect(constantvoltage1.n,arm2.pin_n) annotation(Line(points = {{-80,-70},{-80,-70},{-38.4211,-70},{-38.4211,25.2632},{-38.4211,25.2632}}));
  connect(ground1.p,constantvoltage1.n) annotation(Line(points = {{-80,-90},{-80,-90},{-80,-69.47369999999999},{-80,-69.47369999999999}}));
  connect(chassis1.frame_S,arm3.frame_a) annotation(Line(points = {{0,-26.25},{5.78947,-26.25},{5.78947,-35.7895},{5.78947,-35.7895}}));
  connect(chassis1.frame_N,arm4.frame_a) annotation(Line(points = {{0,26.25},{6.31579,26.25},{6.31579,56.3158},{6.31579,56.3158}}));
  connect(chassis1.frame_W,arm1.frame_a) annotation(Line(points = {{26.25,0},{56.3158,0},{56.3158,7.89474},{56.3158,7.89474}}));
  connect(chassis1.frame_E,arm2.frame_a) annotation(Line(points = {{-26.25,0},{-37.3684,0},{-37.3684,6.84211},{-37.3684,6.84211}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Quadrotor;

