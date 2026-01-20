#
# Trailer path planning library
#
# author: Atsushi Sakai(@Atsushi_twi)
#

@info "trailerlib loaded from" file=@__FILE__


module trailerlib

using PyPlot
using NearestNeighbors

# Vehicle parameter
# const WB = 3.7  #[m] wheel base: rear to front steer
# const LT = 8.0 #[m] rear to trailer wheel
# const W = 2.6 #[m] width of vehicle
# const LF = 4.5 #[m] distance from rear to vehicle front end of vehicle
# const LB = 1.0 #[m] distance from rear to vehicle back end of vehicle
# const LTF = 1.0 #[m] distance from rear to vehicle front end of trailer
# const LTB = 9.0 #[m] distance from rear to vehicle back end of trailer
# const MAX_STEER = 0.6 #[rad] maximum steering angle 
# const TR = 0.5 # Tyre radius [m] for plot
# const TW = 1.0 # Tyre width [m] for plot

# Ver. 비행기
const WB = 3.7  #[m] wheel base: rear to front steer
const LT = 3.0 #힌지-트레일러 기준점 거리를 줄임!
const W = 10.0 #비행기 폭 날개 포함
const LF = 4.5 #[m] distance from rear to vehicle front end of vehicle
const LB = 1.0 #[m] distance from rear to vehicle back end of vehicle
const LTF = 2.0 # 비행기와 토잉카 사이의 거리
const LTB = 9.0 # 연결점부터 비행기 꼬리까지의 거리
const MAX_STEER = 0.6 #[rad] maximum steering angle 
const TR = 0.5 # Tyre radius [m] for plot
const TW = 1.0 # Tyre width [m] for plot

# for collision check
const WBUBBLE_DIST = 3.5 #distance from rear and the center of whole bubble
const WBUBBLE_R = 10.0 # whole bubble radius
const B = 4.45 # distance from rear to vehicle back end
const C = 11.54 # distance from rear to vehicle front end
const I = 8.55 # width of vehicle
const VRX = [C, C, -B, -B, C ]
const VRY = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]

function check_collision(x::Array{Float64},
                         y::Array{Float64},
                         yaw::Array{Float64},
                         kdtree::KDTree,
                         ox::Array{Float64},
                         oy::Array{Float64},
                         wbd::Float64,
                         wbr::Float64,
                         vrx::Array{Float64},
                         vry::Array{Float64})::Bool

    for (ix, iy, iyaw) in zip(x, y, yaw)
        cx = ix + wbd*cos(iyaw)
        cy = iy + wbd*sin(iyaw)

        # Whole bubble check
        ids = inrange(kdtree, [cx, cy], wbr, true)
        # println(length(ids))
        if length(ids) == 0 continue end

        if !rect_check(ix, iy, iyaw, ox[ids], oy[ids], vrx, vry)
            # println("collision")
            return false #collision
        end

        # println(ids)
    end
    return true #OK
end


function rect_check(ix::Float64, iy::Float64, iyaw::Float64,
                    ox::Array{Float64}, oy::Array{Float64},
                    vrx::Array{Float64}, vry::Array{Float64}
                   )::Bool

    c = cos(-iyaw)
    s = sin(-iyaw)

    for (iox, ioy) in zip(ox, oy)
        tx = iox - ix
        ty = ioy - iy
        lx = (c*tx - s*ty)
        ly = (s*tx + c*ty)

        sumangle = 0.0
        for i in 1:length(vrx)-1
            x1 = vrx[i] - lx
            y1 = vry[i] - ly
            x2 = vrx[i+1] - lx
            y2 = vry[i+1] - ly

            d1 = hypot(x1, y1)
            d2 = hypot(x2, y2)

            den = d1 * d2
            if den < 1e-12
                continue
            end

            theta1 = atan(y1, x1)
            tty = (-sin(theta1)*x2 + cos(theta1)*y2)

            tmp = (x1*x2 + y1*y2) / den
            tmp = clamp(tmp, -1.0, 1.0)

            if tty >= 0.0
                sumangle += acos(tmp)
            else
                sumangle -= acos(tmp)
            end
        end

        if abs(sumangle) >= (pi - 1e-6)
            return false # collision
        end
    end

    return true # OK
end
# 로컬 폴리곤(vrx,vry)을 월드 좌표 꼭짓점으로 변환
function poly_world(ix::Float64, iy::Float64, iyaw::Float64,
                    vrx::Array{Float64}, vry::Array{Float64})
    c = cos(iyaw)
    s = sin(iyaw)
    pts = Vector{Tuple{Float64,Float64}}(undef, length(vrx))
    for i in 1:length(vrx)
        xw = ix + c*vrx[i] - s*vry[i]
        yw = iy + s*vrx[i] + c*vry[i]
        pts[i] = (xw, yw)
    end
    return pts
end

function check_collision_debug(x::Array{Float64},
                               y::Array{Float64},
                               yaw::Array{Float64},
                               kdtree::KDTree,
                               ox::Array{Float64},
                               oy::Array{Float64},
                               wbd::Float64,
                               wbr::Float64,
                               vrx::Array{Float64},
                               vry::Array{Float64};
                               label::String = "",
                               max_near::Int = 12)::Bool # check_collision 충돌 시 원인 근처 점 출력
    for (ix, iy, iyaw) in zip(x, y, yaw)
        cx = ix + wbd*cos(iyaw)
        cy = iy + wbd*sin(iyaw)

        ids = inrange(kdtree, [cx, cy], wbr, true)

        if length(ids) == 0
            continue
        end

        # 폴리곤 충돌 검사
        if !rect_check(ix, iy, iyaw, ox[ids], oy[ids], vrx, vry)
            pts = Vector{Tuple{Float64,Float64,Float64}}()
            sizehint!(pts, min(length(ids), max_near))
            for idx in ids
                d = hypot(ox[idx] - cx, oy[idx] - cy)
                push!(pts, (ox[idx], oy[idx], d))
            end
            sort!(pts, by = p -> p[3])
            near = pts[1:min(end, max_near)]

            poly = poly_world(ix, iy, iyaw, vrx, vry)

            @info "[COLLISION][$label]" pose=(ix=ix,iy=iy,yaw=iyaw) bubble=(cx=cx,cy=cy,r=wbr) n_inrange=length(ids) near_points=near poly_world=poly
            return false
        end
    end

    return true
end


function calc_trailer_yaw_from_xyyaw(
                   x::Array{Float64},
                   y::Array{Float64},
                   yaw::Array{Float64},
                   init_tyaw::Float64,
                   steps::Array{Float64})::Array{Float64}
    """
    calc trailer yaw from x y yaw lists
    """

    tyaw = fill(0.0, length(x))
    tyaw[1] = init_tyaw

    for i in 2:length(x)
        tyaw[i] += tyaw[i-1] + steps[i-1]/LT*sin(yaw[i-1] - tyaw[i-1])
    end

    return tyaw
end


function trailer_motion_model(x, y, yaw0, yaw1, D, d, L, delta)
    """
    Motion model for trailer 
    see:
    http://planning.cs.uiuc.edu/node661.html#77556
    """
    x += D*cos(yaw0)
    y += D*sin(yaw0)
    yaw0 += D/L*tan(delta)
    yaw1 += D/d*sin(yaw0 - yaw1)

    return x, y, yaw0, yaw1
end


# function check_trailer_collision(
#                     ox::Array{Float64},
#                     oy::Array{Float64},
#                     x::Array{Float64},
#                     y::Array{Float64},
#                     yaw0::Array{Float64},
#                     yaw1::Array{Float64};
#                     kdtree = nothing
#                    )
#     """
#     collision check function for trailer

#     """

#     if kdtree == nothing
# 		kdtree = KDTree([ox'; oy'])
#     end

#     vrxt = [LTF, LTF, -LTB, -LTB, LTF]
#     vryt = [-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0]

#     # bubble parameter
#     DT = (LTF + LTB)/2.0 - LTB
#     DTR = (LTF + LTB)/2.0 + 0.3 

#     # check trailer
#     if !check_collision(x, y, yaw1, kdtree, ox, oy, DT, DTR, vrxt, vryt)
#         return false
#     end

#     vrxf = [LF, LF, -LB, -LB, LF]
#     vryf = [-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0]
  
#     # bubble parameter
#     DF = (LF + LB)/2.0 - LB
#     DFR = (LF + LB)/2.0 + 0.3 

#     # check front trailer
#     if !check_collision(x, y, yaw0, kdtree, ox, oy, DF, DFR, vrxf, vryf)
#         return false
#     end

#     return true #OK
# end

function check_trailer_collision(
                    ox::Array{Float64},
                    oy::Array{Float64},
                    x::Array{Float64},
                    y::Array{Float64},
                    yaw0::Array{Float64},
                    yaw1::Array{Float64};
                    kdtree = nothing,
                    debug::Bool = false,
                    max_near::Int = 12
                   )
    """
    collision check function for trailer

    """

    if kdtree == nothing
		kdtree = KDTree([ox'; oy'])
    end

    vrxt = [LTF, LTF, -LTB, -LTB, LTF]
    vryt = [-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0]

    # bubble parameter
    DT = (LTF + LTB)/2.0 - LTB
    DTR = (LTF + LTB)/2.0 + 0.3 

    # check trailer
    if debug
        if !check_collision_debug(x, y, yaw1, kdtree, ox, oy, DT, DTR, vrxt, vryt; label="TRAILER", max_near=max_near)
            return false
        end
    else
        if !check_collision(x, y, yaw1, kdtree, ox, oy, DT, DTR, vrxt, vryt)
            return false
        end
    end


    vrxf = [LF, LF, -LB, -LB, LF]
    vryf = [-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0]
  
    # bubble parameter
    DF = (LF + LB)/2.0 - LB
    DFR = (LF + LB)/2.0 + 0.3 

    # check front trailer
    if debug
        if !check_collision_debug(x, y, yaw0, kdtree, ox, oy, DF, DFR, vrxf, vryf; label="TRUCK", max_near=max_near)
            return false
        end
    else
        if !check_collision(x, y, yaw0, kdtree, ox, oy, DF, DFR, vrxf, vryf)
            return false
        end
    end


    return true #OK
end


function plot_trailer(x::Float64,
                      y::Float64,
                      yaw::Float64,
                      yaw1::Float64,
                      steer::Float64)

    truckcolor = "-k"

    LENGTH = LB+LF
    LENGTHt = LTB+LTF

    truckOutLine = [-LB (LENGTH - LB) (LENGTH - LB) (-LB) (-LB);
                    W/2 W/2 -W/2 -W/2 W / 2]
    trailerOutLine = [-LTB (LENGTHt - LTB) (LENGTHt - LTB) (-LTB) (-LTB);
                    W/2 W/2 -W/2 -W/2 W / 2]

    rr_wheel = [TR -TR -TR TR TR;
                -W/12.0+TW  -W/12.0+TW W/12.0+TW W/12.0+TW -W/12.0+TW]
                
    rl_wheel = [TR -TR -TR TR TR;
                -W/12.0-TW  -W/12.0-TW W/12.0-TW W/12.0-TW -W/12.0-TW]

    fr_wheel = [TR -TR -TR TR TR
                -W/12.0+TW  -W/12.0+TW W/12.0+TW W/12.0+TW -W/12.0+TW]
                
    fl_wheel = [TR -TR -TR TR TR;
                -W/12.0-TW  -W/12.0-TW W/12.0-TW W/12.0-TW -W/12.0-TW]
    tr_wheel = [TR -TR -TR TR TR
                -W/12.0+TW  -W/12.0+TW W/12.0+TW W/12.0+TW -W/12.0+TW]
                
    tl_wheel = [TR -TR -TR TR TR;
                -W/12.0-TW  -W/12.0-TW W/12.0-TW W/12.0-TW -W/12.0-TW]
 
    Rot1 = [cos(yaw) sin(yaw);
           -sin(yaw) cos(yaw)]
    Rot2 = [cos(steer) sin(steer);
           -sin(steer) cos(steer)]
    Rot3 = [cos(yaw1) sin(yaw1);
           -sin(yaw1) cos(yaw1)]

    fr_wheel = (fr_wheel' * Rot2)'
    fl_wheel = (fl_wheel' * Rot2)'
    fr_wheel[1,:] .+= WB
    fl_wheel[1,:] .+= WB
    fr_wheel = (fr_wheel' * Rot1)'
    fl_wheel = (fl_wheel' * Rot1)'

    tr_wheel[1,:] .-= LT
    tl_wheel[1,:] .-= LT
    tr_wheel = (tr_wheel' * Rot3)'
    tl_wheel = (tl_wheel' * Rot3)'

    truckOutLine = (truckOutLine' * Rot1)'
    trailerOutLine = (trailerOutLine' * Rot3)'
    rr_wheel = (rr_wheel' * Rot1)'
    rl_wheel = (rl_wheel' * Rot1)'

    truckOutLine[1, :] .+= x
    truckOutLine[2, :] .+= y
    trailerOutLine[1, :] .+= x
    trailerOutLine[2, :] .+= y
    fr_wheel[1, :] .+= x
    fr_wheel[2, :] .+= y
    rr_wheel[1, :] .+= x
    rr_wheel[2, :] .+= y
    fl_wheel[1, :] .+= x
    fl_wheel[2, :] .+= y
    rl_wheel[1, :] .+= x
    rl_wheel[2, :] .+= y

    tr_wheel[1, :] .+= x
    tr_wheel[2, :] .+= y
    tl_wheel[1, :] .+= x
    tl_wheel[2, :] .+= y

    plot(truckOutLine[1, :], truckOutLine[2, :], truckcolor)
    plot(trailerOutLine[1, :], trailerOutLine[2, :], truckcolor)
    plot(fr_wheel[1, :], fr_wheel[2, :], truckcolor)
    plot(rr_wheel[1, :], rr_wheel[2, :], truckcolor)
    plot(fl_wheel[1, :], fl_wheel[2, :], truckcolor)
    plot(rl_wheel[1, :], rl_wheel[2, :], truckcolor)

    plot(tr_wheel[1, :], tr_wheel[2, :], truckcolor)
    plot(tl_wheel[1, :], tl_wheel[2, :], truckcolor)
    plot(x, y, "*")
end


function main()
    x = 0.0
    y = 0.0
    yaw0 = deg2rad(10.0)
    yaw1 = deg2rad(-10.0)

    plot_trailer(x, y, yaw0, yaw1, 0.0)

    DF = (LF + LB)/2.0 - LB
    DFR = (LF + LB)/2.0 + 0.3 

    DT = (LTF + LTB)/2.0 - LTB
    DTR = (LTF + LTB)/2.0 + 0.3 

    axis("equal")

    show()

end
 

if length(PROGRAM_FILE)!=0 &&
	occursin(PROGRAM_FILE, @__FILE__)
    @time main()
end

end

