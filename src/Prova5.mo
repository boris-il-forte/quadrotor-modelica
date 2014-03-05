model Prova5
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-80,60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1) annotation(Placement(visible = true, transformation(origin = {20,40}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-80,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder1 annotation(Placement(visible = true, transformation(origin = {-40,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1d1 annotation(Placement(visible = true, transformation(origin = {0,-20}, extent = {{10,-10},{-10,10}}, rotation = -90)));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(useSupport = true, Jr = 1) annotation(Placement(visible = true, transformation(origin = {-20,40}, extent = {{-15,-15},{15,15}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantvoltage1(V = 12) annotation(Placement(visible = true, transformation(origin = {-40,80}, extent = {{10,-10},{-10,10}}, rotation = 0)));
equation
  connect(bodycylinder1.frame_a,world.frame_b) annotation(Line(points = {{-50,-20},{-70.4268,-20},{-70.4268,-18.2927},{-70.4268,-18.2927}}));
  connect(ground1.p,constantvoltage1.n) annotation(Line(points = {{-80,70},{-79.878,70},{-79.878,80.1829},{-50,80},{-50,80}}));
  connect(constantvoltage1.p,dcpm.pin_ap) annotation(Line(points = {{-30,80},{-30.4878,80},{-11,55},{-11,55}}));
  connect(constantvoltage1.n,dcpm.pin_an) annotation(Line(points = {{-50,80},{-50,80},{-29,55},{-29,55}}));
  connect(dcpm.support,mounting1d1.flange_b) annotation(Line(points = {{-5,25},{0,25},{0,-10.061},{0,-10.061}}));
  connect(inertia1.flange_b,dcpm.flange) annotation(Line(points = {{10,40},{-25,40},{-5,40},{-5,40}}));
  connect(mounting1d1.frame_a,bodycylinder1.frame_b) annotation(Line(points = {{-10,-20},{-10,-20.122},{-50,-20},{-30,-20}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Prova5;