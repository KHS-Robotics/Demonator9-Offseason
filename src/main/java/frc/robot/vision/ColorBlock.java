package frc.robot.vision;

import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class ColorBlock {
	private double size, distance, x, y, width, height;
	private int sig;

	public final static double xHalf = (315.0 / 2.0);
	public final static double yHalf = (207.0 / 2.0);
	// (0, 0) --> (315, 0)
	// (0, 0) --> (0, 207)

	public ColorBlock(double x, double y, double width, double height, int sig) {
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.sig = sig;
		update();
	}

	public ColorBlock(Block block) {
		x = block.getX();
		y = block.getY();
		width = block.getWidth();
		height = block.getHeight();
		sig = block.getSignature();
		update();
	}

	public void update() {
		size = calcSize();
		distance = calcDist();
	}

	public double calcDist() {
		double totalDist = Math.sqrt(Math.pow((x - xHalf), 2) + Math.pow((y - yHalf), 2));
		return totalDist;
	}

	public double calcSize() {
		return width * height;
	}

	public double getSize(ColorBlock args) {
		return args.getWidth() * args.getLength();
	}

	public double getSize() {
		return size;
	}

	public double getDist() {
		return distance;
	}

	public double getXDist() {
		return xHalf - x;
	}

	public double getLength() {
		return height;
	}

	public double getWidth() {
		return width;
	}

	public int getSig() {
		return sig;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
}