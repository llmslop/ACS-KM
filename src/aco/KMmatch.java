package aco;

import java.util.ArrayList;

//Maximum weight, take n = -n to get minimum
public class KMmatch {
	
	public int pop;
	public int [][]w;
	
	private int []x;
	private int []y;
	private int []prev_x;
	private int []prev_y;
	
	public int []son_y;
	
	private int []slack;
	private int []par;
	
	public void resize(int n)
	{
		pop = n;
		w = new int [n+1][n+1];
		x = new int [n+1];
		y = new int [n+1];
		prev_x = new int [n+1];
		prev_y = new int [n+1];
		
		son_y = new int [n+1];
		
		slack = new int [n+1];
		par = new int [n+1];
	}
	
	private void adjust(int v){
		son_y[v] = prev_y[v];
		if (prev_x[son_y[v]] != -2)
			adjust(prev_x[son_y[v]]);
	}
	
	private boolean find(int v){
		int i;
		for (i = 0; i < pop; i++)
			if (prev_y[i] == -1){
				if (slack[i]>x[v] + y[i] - w[v][i]){
					slack[i] = x[v] + y[i] - w[v][i];
					par[i] = v;
				}
				if (x[v] + y[i] == w[v][i]){
					prev_y[i] = v;
					if (son_y[i] == -1){
						adjust(i);
						return true;
					}
					if (prev_x[son_y[i]] != -1)
						continue;
					prev_x[son_y[i]] = i;
					if (find(son_y[i]))
						return true;
				}
			}
		
		return false;
	}
	public int km(){
		int i, j, m;
		for (i = 0; i < pop; i++){
			son_y[i] = -1;
			y[i] = 0;
		}
		for (i = 0; i < pop; i++){
			x[i] = 0;
			for (j = 0; j < pop; j++){
				x[i] = Math.max(x[i], w[i][j]);
			}
		}
		boolean flag;
		for (i = 0; i < pop; i++){
			for (j = 0; j < pop; j++){
				prev_x[j] = prev_y[j] = -1;
				slack[j] = 1000000000;
			}
			prev_x[i] = -2;
			if (find(i))continue;
			flag = false;
			while (!flag){
				m = 1000000000;
				for (j = 0; j < pop; j++)
				if (prev_y[j] == -1)
					m = Math.min(m, slack[j]);
				for (j = 0; j < pop; j++){
					if (prev_x[j] != -1)
						x[j] -= m;
					if (prev_y[j] != -1)
						y[j] += m;
					else
						slack[j] -= m;
				}


				for (j = 0; j < pop; j++)
				if (prev_y[j] == -1 && slack[j]==0){
					prev_y[j] = par[j];
					if (son_y[j] == -1){
						adjust(j);
						flag = true;
						break;
					}
					prev_x[son_y[j]] = j;
					if (find(son_y[j])){
						flag = true;
						break;
					}
				}
			}
		}
		int ans = 0;

		for (int is = 0; is < pop; is++)
			ans += w[son_y[is]][is];
		return ans;
	}
	
	//test main
	public static void main(String[] args) {
		KMmatch km = new KMmatch();
		km.resize(3);
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				km.w[i][j] = 0;
			}
		}
		km.w[1][2] = 3;
		//km.w[2][0] = -1;
		//km.w[0][0] = -1;
		System.out.println("ans: "+ km.km());
		
		System.out.println("Match record");
		for(int i=0;i<3;i++)
		{
			System.out.println("x y: " + km.son_y[i] + " " + i);
		}
		
		//test Arr
		ArrayList<Integer> v = new ArrayList();
		v.add(23);
		System.out.println("arrsize: "+v.size());
		v.clear();
		System.out.println("arrsize: "+v.size());
		v.add(23);
		System.out.println("arrsize: "+v.size());
	}
}
