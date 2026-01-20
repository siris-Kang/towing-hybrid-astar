using HTTP
using JSON3
using Dates
using Random
using NearestNeighbors

include("trailerlib.jl")
include("rs_path.jl")
include("grid_a_star.jl")
include("trailer_hybrid_a_star.jl")


function parse_target(target::AbstractString)
    s = String(target)
    parts = split(s, '?'; limit=2)
    path = parts[1]

    q = Dict{String,String}()
    if length(parts) == 2
        for kv in split(parts[2], '&')
            isempty(kv) && continue
            kvp = split(kv, '='; limit=2)
            k = kvp[1]
            v = length(kvp) == 2 ? kvp[2] : ""
            q[k] = v
        end
    end
    return path, q
end

getf(q::Dict{String,String}, key::String, default::Float64) =
    haskey(q, key) ? parse(Float64, q[key]) : default

function fail_response(msg::String)
    return Dict("ok" => false, "error" => msg)
end

function path_to_dict(p::trailer_hybrid_a_star.Path)
    dir_int = [d ? 1 : -1 for d in p.direction]
    return Dict(
        "ok" => true,
        "length" => length(p.x),
        "x" => p.x,
        "y" => p.y,
        "yaw" => p.yaw,
        "yaw1" => p.yaw1,
        "direction" => dir_int,
        "cost" => p.cost
    )
end

function save_plan_log(req_dict::Dict, resp_dict::Dict; base_dir="planner_logs")
    mkpath(base_dir)
    run_id = Dates.format(now(), "yyyymmdd_HHMMSS") * "_" * randstring(4)
    run_dir = joinpath(base_dir, run_id)
    mkpath(run_dir)

    open(joinpath(run_dir, "request.json"), "w") do io
        write(io, JSON3.write(req_dict))
    end
    open(joinpath(run_dir, "response.json"), "w") do io
        write(io, JSON3.write(resp_dict))
    end

    return run_dir
end


function handle(req::HTTP.Request)
    try
        path, q = parse_target(req.target)
        
        if path == "/health"
            return HTTP.Response(200, "ok")
        end
        if !(path in ["/plan", "/check"])
            return HTTP.Response(404, "not found")
        end
        if path == "/check"
            body = JSON3.write(Dict(
                "ok" => true,
                "ok_start" => ok_start,
                "ok_goal" => ok_goal,
                "sx" => sx, "sy" => sy,
                "gx" => gx, "gy" => gy
            ))
            return HTTP.Response(200, ["Content-Type" => "application/json"], body)
        end

        sx    = getf(q, "sx",  14.0);  sy = getf(q, "sy", 10.0)
        syaw  = getf(q, "syaw", 0.0);  styaw = getf(q, "styaw", 0.0)
        gx    = getf(q, "gx",  0.0);   gy = getf(q, "gy", 0.0)
        gyaw  = getf(q, "gyaw", 0.0);  gtyaw = getf(q, "gtyaw", 0.0)

        xyreso  = trailer_hybrid_a_star.XY_GRID_RESOLUTION
        yawreso = trailer_hybrid_a_star.YAW_GRID_RESOLUTION

        ox = Float64[]
        oy = Float64[]

        if req.method == "POST" && !isempty(req.body)
            data = JSON3.read(String(req.body))
            sx    = Float64(get(data, :sx, sx));     sy    = Float64(get(data, :sy, sy))
            syaw  = Float64(get(data, :syaw, syaw)); styaw = Float64(get(data, :styaw, styaw))
            gx    = Float64(get(data, :gx, gx));     gy    = Float64(get(data, :gy, gy))
            gyaw  = Float64(get(data, :gyaw, gyaw)); gtyaw = Float64(get(data, :gtyaw, gtyaw))

            xyreso  = Float64(get(data, :xyreso, xyreso))
            yawreso = Float64(get(data, :yawreso, yawreso))

            if haskey(data, :ox) && haskey(data, :oy)
                ox = Float64.(collect(data[:ox]))
                oy = Float64.(collect(data[:oy]))
            end
        end

        # 장애물 위치 체크
        kdtree = NearestNeighbors.KDTree([ox'; oy'])

        ok_start = trailerlib.check_trailer_collision(ox, oy, [sx], [sy], [syaw], [styaw])
        if !ok_start
            trailerlib.check_trailer_collision(ox, oy, [sx], [sy], [syaw], [styaw]; debug=true, max_near=12)
        end
        ok_goal  = trailerlib.check_trailer_collision(ox, oy, [gx], [gy], [gyaw], [gtyaw], kdtree=kdtree)
        if ok_goal
            trailerlib.check_trailer_collision(ox, oy, [gx], [gy], [gyaw], [gtyaw]; debug=true, max_near=12)
        end
        @info "[COLL CHECK]" ok_start=ok_start ok_goal=ok_goal sx=sx sy=sy gx=gx gy=gy

        p = trailer_hybrid_a_star.calc_hybrid_astar_path(
            sx, sy, syaw, styaw,
            gx, gy, gyaw, gtyaw,
            ox, oy,
            xyreso, yawreso
        )

        out = if p isa trailer_hybrid_a_star.Path
            path_to_dict(p)
        else
            fail_response("Cannot find path (No open set). Check start/goal/obstacles.")
        end

        # LOG 저장
        req_snapshot = Dict(
            "sx"=>sx, "sy"=>sy, "syaw"=>syaw, "styaw"=>styaw,
            "gx"=>gx, "gy"=>gy, "gyaw"=>gyaw, "gtyaw"=>gtyaw,
            "xyreso"=>xyreso, "yawreso"=>yawreso,
            "ox"=>ox, "oy"=>oy
        )

        run_dir = save_plan_log(req_snapshot, out)
        @info "saved planner log" run_dir=run_dir

        body = JSON3.write(out)
        return HTTP.Response(200, ["Content-Type" => "application/json"], body)

    catch e
        @error "planner error" exception=(e, catch_backtrace())
        body = JSON3.write(Dict("ok"=>false, "error"=>string(e)))
        return HTTP.Response(500, ["Content-Type"=>"application/json"], body)
    end
end


Base.exit_on_sigint(true)

try
    println("Planner server on http://0.0.0.0:8080")
    HTTP.serve(handle, "0.0.0.0", 8080; verbose=false)
catch e
    if e isa InterruptException
        @info "SIGINT received. Shutting down."
    else
        rethrow()
    end
end