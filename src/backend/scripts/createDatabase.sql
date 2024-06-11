-- Create the Robot table
CREATE TABLE IF NOT EXISTS Robot (
  id int PRIMARY KEY AUTO_INCREMENT,
  last_manufactured text NOT NULL
);

-- Create the Unit table
CREATE TABLE IF NOT EXISTS Unit (
  id int PRIMARY KEY AUTO_INCREMENT,
  city text,
  state text
);

-- Create the Reboiler table
CREATE TABLE IF NOT EXISTS Reboiler (
  id int PRIMARY KEY AUTO_INCREMENT,
  number int NOT NULL,
  unit_id int,
  started_at text,
  finished_at text,
  FOREIGN KEY (unit_id) REFERENCES Unit(id)
);

-- Create the Order table
CREATE TABLE IF NOT EXISTS Order (
  id int PRIMARY KEY AUTO_INCREMENT,
  status text,
  robot_id int,
  reboiler_id int,
  started_at text,
  finished_at text,
  FOREIGN KEY (robot_id) REFERENCES Robot(id),
  FOREIGN KEY (reboiler_id) REFERENCES Reboiler(id)
);

-- Create the Image table
CREATE TABLE IF NOT EXISTS Image (
  id int PRIMARY KEY AUTO_INCREMENT,
  image text NOT NULL,
  taken_at text NOT NULL
);

-- Create the Examination table
CREATE TABLE IF NOT EXISTS Examination (
  id int PRIMARY KEY AUTO_INCREMENT,
  step text NOT NULL,
  started_at text NOT NULL,
  finished_at text NOT NULL,
  order_id int,
  FOREIGN KEY (order_id) REFERENCES Order(id)
);

-- Create the TubeState table
CREATE TABLE IF NOT EXISTS TubeState (
  id int PRIMARY KEY AUTO_INCREMENT,
  dirtness bool NOT NULL,
  image_id int,
  examination_id int,
  reboiler_id int,
  FOREIGN KEY (image_id) REFERENCES Image(id),
  FOREIGN KEY (examination_id) REFERENCES Examination(id),
  FOREIGN KEY (reboiler_id) REFERENCES Tube(id)
);
