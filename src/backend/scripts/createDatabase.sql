-- Create the Unit table
CREATE TABLE IF NOT EXISTS `Unit` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `city` TEXT,
  `state` TEXT
);

-- Create the Robot table
CREATE TABLE IF NOT EXISTS `Robot` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `nickname` TEXT 
  `unit_id` INTEGER,
  FOREIGN KEY (`unit_id`) REFERENCES `Unit`(`id`)
);

-- Create the Reboiler table
CREATE TABLE IF NOT EXISTS `Reboiler` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `number` INTEGER NOT NULL,
  `unit_id` INTEGER,
  FOREIGN KEY (`unit_id`) REFERENCES `Unit`(`id`)
);

-- Create the Order table
CREATE TABLE IF NOT EXISTS `Order` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `status` TEXT,
  `robot_id` INTEGER,
  `reboiler_id` INTEGER,
  `started_at` INTEGER,
  `finished_at` INTEGER,
  FOREIGN KEY (`robot_id`) REFERENCES `Robot`(`id`),
  FOREIGN KEY (`reboiler_id`) REFERENCES `Reboiler`(`id`)
);

-- Create the Image table
CREATE TABLE IF NOT EXISTS `Image` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `image` TEXT NOT NULL,
  `taken_at` INTEGER NOT NULL
);

-- Create the Examination table
CREATE TABLE IF NOT EXISTS `Examination` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `step` TEXT NOT NULL,
  `started_at` INTEGER NOT NULL,
  `finished_at` INTEGER ,
  `order_id` INTEGER,
  FOREIGN KEY (`order_id`) REFERENCES `Order`(`id`)
);

-- Create the TubeState table
CREATE TABLE IF NOT EXISTS `TubeState` (
  `id` INTEGER PRIMARY KEY AUTOINCREMENT,
  `dirtness` BOOLEAN NOT NULL,
  `image_id` INTEGER,
  `examination_id` INTEGER,
  FOREIGN KEY (`image_id`) REFERENCES `Image`(`id`),
  FOREIGN KEY (`examination_id`) REFERENCES `Examination`(`id`)
);
