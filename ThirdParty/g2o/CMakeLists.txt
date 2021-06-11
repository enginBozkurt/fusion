
aux_source_directory(./core G2O_CORE_SRC)
aux_source_directory(./solvers G2O_SOLVERS_SRC)
aux_source_directory(./stuff G2O_STUFF_SRC)
aux_source_directory(./types G2O_TYPES_SRC)

add_library(g2o STATIC ${G2O_CORE_SRC} ${G2O_SOLVERS_SRC} ${G2O_STUFF_SRC} ${G2O_TYPES_SRC})