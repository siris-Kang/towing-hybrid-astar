using HTTP
using JSON3

include("trailerlib.jl")
include("rs_path.jl")
include("grid_a_star.jl")
include("trailer_hybrid_a_star.jl")


function default_obstacles()
    ox = Float64[]
    oy = Float64[]

    for i in -25.0:1.0:25.0
        push!(ox, i); push!(oy, -15.0)
        push!(ox, i); push!(oy,  15.0)
    end
    for j in -15.0:1.0:15.0
        push!(ox, -25.0); push!(oy, j)
        push!(ox,  25.0); push!(oy, j)
    end

    for i in -5.0:1.0:5.0
        push!(ox, i); push!(oy, 0.0)
    end
    return ox, oy
end

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

function path_to_dict(p)
    dir_int = [d ? 1 : -1 for d in p.direction]
    Dict(
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


function handle(req::HTTP.Request)
    try
        path, q = parse_target(req.target)

        if path == "/health"
            return HTTP.Response(200, "ok")
        end

        if path != "/plan"
            return HTTP.Response(404, "not found")
        end

        sx    = getf(q, "sx",  14.0)
        sy    = getf(q, "sy",  10.0)
        syaw  = getf(q, "syaw", 0.0)
        styaw = getf(q, "styaw", 0.0)

        gx    = getf(q, "gx",  0.0)
        gy    = getf(q, "gy",  0.0)
        gyaw  = getf(q, "gyaw", 0.0)
        gtyaw = getf(q, "gtyaw", 0.0)

        ox, oy = default_obstacles()

        p = trailer_hybrid_a_star.calc_hybrid_astar_path(
            sx, sy, syaw, styaw,
            gx, gy, gyaw, gtyaw,
            ox, oy,
            trailer_hybrid_a_star.XY_GRID_RESOLUTION,
            trailer_hybrid_a_star.YAW_GRID_RESOLUTION
        )


        body = JSON3.write(path_to_dict(p))

        return HTTP.Response(200, ["Content-Type" => "application/json"], body)

    catch e
        @error "planner error" exception=(e, catch_backtrace())
        body = JSON3.write(Dict("ok"=>false, "error"=>string(e)))
        return HTTP.Response(500, ["Content-Type"=>"application/json"], body)
    end

end

println("Planner server on http://0.0.0.0:8080")
HTTP.serve(handle, "0.0.0.0", 8080)
