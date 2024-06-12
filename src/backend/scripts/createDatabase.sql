-- Create the Unit table
CREATE TABLE IF NOT EXISTS Unit (
    id int PRIMARY KEY AUTO_INCREMENT,
    city text NOT NULL,
    state text NOT NULL
);

-- Create the Reboiler table
CREATE TABLE IF NOT EXISTS Reboiler (
    id int PRIMARY KEY AUTO_INCREMENT,
    number int NOT NULL,
    unit_id int,
    FOREIGN KEY (unit_id) REFERENCES Unit(id)
);

-- Create the Robot table
CREATE TABLE IF NOT EXISTS Robot (
    id int PRIMARY KEY AUTO_INCREMENT,
    last_manufactured int NOT NULL
);

-- Create the Tube table
CREATE TABLE IF NOT EXISTS Tube (
    id int PRIMARY KEY AUTO_INCREMENT,
    reboiler_id int,
    position_column int NOT NULL,
    position_row int NOT NULL,
    FOREIGN KEY (reboiler_id) REFERENCES Reboiler(id)
);

-- Create the Image table
CREATE TABLE IF NOT EXISTS Image (
    id int PRIMARY KEY AUTO_INCREMENT,
    image text NOT NULL,
    taken_at int NOT NULL
);

-- Create the Examination table
CREATE TABLE IF NOT EXISTS Examination (
    id int PRIMARY KEY AUTO_INCREMENT,
    etapa text NOT NULL,
    robot_id int,
    reboiler_id int,
    started_at int NOT NULL,
    finished_at int NOT NULL,
    FOREIGN KEY (robot_id) REFERENCES Robot(id),
    FOREIGN KEY (reboiler_id) REFERENCES Reboiler(id)
);

-- Create the TubeState table
CREATE TABLE IF NOT EXISTS TubeState (
    id int PRIMARY KEY AUTO_INCREMENT,
    dirtness int NOT NULL,
    image_id int,
    examination_id int,
    tube_id int,
    FOREIGN KEY (image_id) REFERENCES Image(id),
    FOREIGN KEY (examination_id) REFERENCES Examination(id),
    FOREIGN KEY (tube_id) REFERENCES Tube(id)
);
