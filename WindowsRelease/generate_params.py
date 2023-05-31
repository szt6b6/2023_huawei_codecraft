to_sell_need_frames_error = [10]#[ i for i in range(-10, 40, 10)]
to_buy_need_frames_error = [10]#[ i for i in range(-10, 30, 10)]
sell_bench_logical_dist_estimate = [500.0] #[-150, 0, 150]
count_have_materials_weight = [-10] #[ i for i in range(-50,50,10)]
produce_done_status_weight = [0] #[ i for i in range(-50, 50, 20)]
limit_bench_to_buy = [7] #[7, 8]
average_speed = [0.10]
bench_out_edge_num_weight = [-0.2]#[i/10.0 for i in range(-10, 10, 1)]
bench_in_edge_num_weight = [-0.7]#[i/10.0 for i in range(-10, 10, 1)]
material_demand_weight = [0] #[i/10.0 for i in range(-10, 10 ,2)]


for i in to_sell_need_frames_error: 
  for j in to_buy_need_frames_error: 
    for k in sell_bench_logical_dist_estimate:
      for m in produce_done_status_weight:
        for n in limit_bench_to_buy: 
          for o in average_speed: 
            for p in bench_out_edge_num_weight:
              for q in bench_in_edge_num_weight:
                for r in material_demand_weight:
                    # code logic here
                  print(i,j,k,m,n,o,p,q,r)


# in cmd: python generate_params.py >> params.txt