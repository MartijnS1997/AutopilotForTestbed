package internal.Testbed;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import internal.Helper.Vector;

public class Block implements WorldObject {
	
	public Block(Vector position) {
		this.position = position;
	}

	@Override
	public void toNextState(float deltaTime) throws IOException {
		// do nothing
	}

	@Override
	public Vector getPosition() {
		return this.position;
	}




	/**
	 * getter for the is visited flag
	 * @return
	 */
	public boolean isVisited(){
		return this.isVisited;
	}

	/**
	 * set the is visited flag to true
	 */
	public void setVisited(){
		this.isVisited = true;
	}

	@Override
	public String toString() {
		return "Block{" +
				"position=" + position +
				", isVisited=" + isVisited +
				'}';
	}

	private Vector position;


	/**
	 * Flag that stores if the block was visited by the drone.
	 */
	private boolean isVisited = false;

	/**
	 * variable used for the max allowed error on the position between de block and the cube
	 */
	private final static float maxPosDifference = 1E-6f;

	private final static String INVALID_CUBE = "the provided cube does not have the same position as the block";
}
