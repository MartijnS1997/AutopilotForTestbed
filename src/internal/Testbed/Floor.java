package internal.Testbed;

import java.io.IOException;

import internal.Helper.Vector;

public class Floor implements WorldObject {

	private Vector position;
	

	public Floor(Vector position) {
		int n = 51; // n moet oneven zijn
        float nx = 20f;
        float nz = 20f;
		this.position = new Vector(-n*nx/2, 0, -n*nz + 50f);
		createFloor(n, nx, nz);
	}
	
	@Override
	public void toNextState(float deltaTime) throws IOException {
		// do nothing
	}

	@Override
	public Vector getPosition() {
		return this.position;
	}


	public void createFloor(int n, float nx, float nz) {
		
        for (int i = 0; i < 2*n*n; i++) {
        	Vector delta = new Vector(nx*(i%n), 0, nz*(i/n));
        	Vector position = delta.vectorSum(getPosition());
            Vector color = new Vector((60.0f+(i%2)*60), 1, 0.6f);
        }
        
//        Tile tile = new Tile(this.getPosition().convertToVector3f(), (new Vector(240, 1, 1f)).convertToVector3f());
//    	tile.setSize(new Vector(20, 0, 20));
//    	this.setAssociatedGraphicsObject(tile);
        Vector position = getPosition();
        Vector color = new Vector((60.0f), 1, 0.6f);
	}
}